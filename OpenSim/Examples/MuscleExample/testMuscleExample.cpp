// testMuscleExample.cpp

/* Copyright (c)  2009 Stanford University
 * Use of the OpenSim software in source form is permitted provided that the following
 * conditions are met:
 *   1. The software is used only for non-commercial research and education. It may not
 *     be used in relation to any commercial activity.
 *   2. The software is not distributed or redistributed.  Software distribution is allowed 
 *     only through https://simtk.org/home/opensim.
 *   3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
 *      presentations, or documents describing work in which OpenSim or derivatives are used.
 *   4. Credits to developers may not be removed from executables
 *     created from modifications of the source.
 *   5. Modifications of source code must retain the above copyright notice, this list of
 *     conditions and the following disclaimer. 
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 *  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 *  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 *  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 *  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Cassidy Kelly

//==============================================================================
//==============================================================================

#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace std;

int main()
{
	try {
		Storage result1("tugOfWar_fatigue_states.sto"), standard1("std_tugOfWar_fatigue_states.sto");
		result1.checkAgainstStandard(standard1, Array<double>(10., 24), __FILE__, __LINE__, "tugOfWar fatigue states failed");
		cout << "tugOfWar fatigue states passed\n";

		Storage result2("tugOfWar_fatigue_states_degrees.mot"), standard2("std_tugOfWar_fatigue_states_degrees.mot");
		result2.checkAgainstStandard(standard2, Array<double>(100., 24), __FILE__, __LINE__, "tugOfWar fatigue states degrees failed");
		cout << "tugOfWar fatigue states degrees passed\n";

		Storage result3("tugOfWar_forces.mot"), standard3("std_tugOfWar_forces.mot");
		result3.checkAgainstStandard(standard3, Array<double>(1000., 24), __FILE__, __LINE__, "tugOfWar forces failed");
		cout << "tugOfWar forces passed\n";
	}
	catch (const Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}