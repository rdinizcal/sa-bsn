#include "libbsn/generator/Markov.hpp"

using namespace std;
using namespace bsn::range;

namespace bsn {
    namespace generator {
        Markov::Markov() : transitions(), currentState(), states() {}

        // Construtor
        Markov::Markov(array<float,25> t, array<Range, 5> r, int32_t initialState) :
            transitions(t),
            currentState(initialState),
            states(r) {}

        Markov::Markov(const Markov &obj) :
            transitions(obj.transitions),
            currentState(obj.currentState),
            states(obj.states) {}
        
        Markov& Markov::operator=(const Markov &obj) {
            transitions = obj.transitions;
            currentState = obj.currentState;
            states = obj.states;
            return (*this);
        }

        const string Markov::toString() const {
            stringstream sstr;

            sstr << "Markov " << "" << endl;

            return sstr.str();
        }

    }
}
