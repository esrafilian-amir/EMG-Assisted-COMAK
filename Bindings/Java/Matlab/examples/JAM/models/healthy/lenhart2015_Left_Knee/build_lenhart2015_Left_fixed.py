"""Builds lenhart2015_Left_fixed.osim (left-only knee model: the detailed left knee is the exact mirror of the detailed
right knee of lenhart2015.osim; the right knee is the simple knee of lenhart2015_Left.osim) and
lenhart2015_Left_fixed_contact_tuned.osim (contact tuning of the dual knee model: tune() of
../lenhart2015_BothKnees/build_lenhart2015_BothKnees.py, imported).

The left knee of lenhart2015_Left.osim is not a mirror of the right knee of lenhart2015.osim (see the header of
build_lenhart2015_BothKnees.py): this script applies the same fixes to the left knee elements of this model:
  - patella joint pf_l: the extra offset frame femur_distal_l_offset (z = -0.008 m) is removed, the default of
    pf_tz_l is minus the default of pf_tz_r;
  - the left cartilage meshes: variable thickness and 5 MPa like the right ones;
  - the contact forces tf_contact and pf_contact: nonlinear, not lumped like the right ones;
  - the patellar tendon PT5 slack length, the viscosity of the springs knee_add_l, knee_rot_l and pf_flex_l.
Every other left knee element is copied from the right one when it differs (the leaf values, except attachment points).
Names in this model: the left forces have no prefix (tf_contact, pf_contact, MCLd1, ...), as in lenhart2015.osim.

Usage: python build_lenhart2015_Left_fixed.py   (the base is lenhart2015_Left.osim; the output files are overwritten)
"""
import importlib.util
import os
import re
import xml.etree.ElementTree as ET

import opensim as osim

HERE = os.path.dirname(os.path.abspath(__file__))
HEALTHY = os.path.dirname(HERE)
BASE = os.path.join(HERE, 'lenhart2015_Left.osim')
RIGHT_MODEL = os.path.join(HEALTHY, 'lenhart2015', 'lenhart2015.osim')
OUT = os.path.join(HERE, 'lenhart2015_Left_fixed.osim')
OUT_TUNED = os.path.join(HERE, 'lenhart2015_Left_fixed_contact_tuned.osim')
RESERVES_RIGHT = os.path.join(HEALTHY, 'lenhart2015', 'lenhart2015_reserve_actuators.xml')
OUT_RESERVES = os.path.join(HERE, 'lenhart2015_Left_fixed_reserve_actuators.xml')

_spec = importlib.util.spec_from_file_location(
    'build_both', os.path.join(HEALTHY, 'lenhart2015_BothKnees', 'build_lenhart2015_BothKnees.py'))
both = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(both)
FLIP, objects, copy_right_to_left, tune = both.FLIP, both.objects, both.copy_right_to_left, both.tune


def patch(base_file, work_file):
    log = []
    tree = ET.parse(base_file)
    root = tree.getroot()
    rroot = ET.parse(RIGHT_MODEL).getroot()

    # 1. contact meshes (the names are the same in the two models)
    rm = objects(rroot, 'ContactGeometrySet')
    lm = objects(root, 'ContactGeometrySet')
    for n in ('femur_cartilage', 'tibia_cartilage', 'patella_cartilage'):
        copy_right_to_left(rm[n], lm[n], ('/mesh_file', '/mesh_back_file'), log, n)

    # 2. forces: contact forces, ligaments, coordinate springs (everything except the muscles)
    rf = objects(rroot, 'ForceSet')
    lf = objects(root, 'ForceSet')
    for name, el in lf.items():
        if re.match(r'^(knee|pf)_[a-z]+_l$', name):
            rname = name[:-2] + '_r'
        else:
            rname = name
        if el.tag == 'Millard2012EquilibriumMuscle' or rname not in rf or rf[rname].tag != el.tag:
            continue
        if rname == name and name.endswith('_l'):
            continue
        copy_right_to_left(rf[rname], el, ('/location', '/visible'), log, name)

    # 3. patella joint: no offset frame, pf_tz default with the mirror sign
    joints = objects(root, 'JointSet')
    pf = joints['pf_l']
    frames = pf.find('frames')
    if frames is not None:
        for fr in list(frames):
            if fr.get('name') == 'femur_distal_l_offset':
                frames.remove(fr)
                log.append('pf_l: removed the offset frame femur_distal_l_offset (z = -0.008)')
        if len(list(frames)) == 0:
            pf.remove(frames)
    sp = pf.find('socket_parent_frame')
    if sp.text.strip() != '/bodyset/femur_distal_l':
        log.append('pf_l: socket_parent_frame %s -> /bodyset/femur_distal_l' % sp.text.strip())
        sp.text = '/bodyset/femur_distal_l'

    # 4. coordinates of the left knee: default values and ranges (sign * right)
    rjoints = objects(rroot, 'JointSet')
    for jn in ('knee', 'pf'):
        rc = {c.get('name'): c for c in rjoints[jn + '_r'].find('coordinates')}
        lc = {c.get('name'): c for c in joints[jn + '_l'].find('coordinates')}
        for base, sgn in FLIP.items():
            if base + '_r' not in rc:
                continue
            r, l = rc[base + '_r'], lc[base + '_l']
            dr = float(r.find('default_value').text)
            rr = [float(x) for x in r.find('range').text.split()]
            want = sgn * dr
            exp = sorted([sgn * rr[0], sgn * rr[1]])
            if abs(float(l.find('default_value').text) - want) > 1e-12:
                log.append('%s_l: default %s -> %.10g' % (base, l.find('default_value').text.strip(), want))
                l.find('default_value').text = repr(want) if want != 0 else '0'
            lr = [float(x) for x in l.find('range').text.split()]
            if max(abs(lr[0] - exp[0]), abs(lr[1] - exp[1])) > 1e-12:
                log.append('%s_l: range %s -> %s' % (base, l.find('range').text.strip(), exp))
                l.find('range').text = '%.10g %.10g' % (exp[0], exp[1])
    tree.write(work_file, xml_declaration=True, encoding='UTF-8')
    return log


def swap_side(name):
    """hip_flex_r_reserve -> hip_flex_l_reserve, pf_l_r3_reserve -> pf_r_r3_reserve (tokens r and l swapped)"""
    return '_'.join({'r': 'l', 'l': 'r'}.get(t, t) for t in name.split('_'))


def write_reserves():
    """Reserve actuators of this model: those of lenhart2015_reserve_actuators.xml with the sides swapped (the
    secondary coordinates of the knee and patella are the left ones, the simple right knee has pf_r_r3, pf_r_tx, pf_r_ty)."""
    text = open(RESERVES_RIGHT, encoding='utf-8').read()
    text = re.sub(r'(<CoordinateActuator name=")([^"]*)(")', lambda m: m.group(1) + swap_side(m.group(2)) + m.group(3), text)
    text = re.sub(r'(<coordinate>)([^<]*)(</coordinate>)', lambda m: m.group(1) + swap_side(m.group(2)) + m.group(3), text)
    with open(OUT_RESERVES, 'w', encoding='utf-8', newline='') as fh:
        fh.write(text)
    print('wrote', OUT_RESERVES)


def main():
    osim.Logger.setLevelString('Error')
    tmp_work = os.path.join(HERE, '_work.osim')
    try:
        log = patch(BASE, tmp_work)
        print('\n'.join(log))
        print('%d changes' % len(log))
        model = osim.Model(tmp_work)
        model.setName('lenhart2015_Left_fixed')
        model.initSystem()
        model.printToXML(OUT)
        print('wrote', OUT)
        m2 = osim.Model(OUT)
        tune(m2)
        m2.initSystem()
        m2.printToXML(OUT_TUNED)
        print('wrote', OUT_TUNED)
        write_reserves()
    finally:
        if os.path.exists(tmp_work):
            os.remove(tmp_work)


if __name__ == '__main__':
    main()
