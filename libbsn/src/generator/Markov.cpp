#include "generator/Markov.hpp"

using namespace std;
using namespace bsn::range;

namespace bsn {
    namespace generator {
        // Gerador de seeds
        random_device rd;
        // Seed para os numeros aleatorios
        mt19937 seed(rd());
        // Gerador de inteiros aleatorios
        uniform_int_distribution<int> probabilityGenerator(1,100);

        Markov::Markov() : transitions(), currentState(), ranges(){}

        // Construtor
        Markov::Markov(array<float,25> t, array<Range, 5> r, int32_t initialState) :
            transitions(t),
            currentState(initialState),
            ranges(r) {}

        Markov::Markov(const Markov &obj) :
            transitions(obj.transitions),
            currentState(obj.currentState),
            ranges(obj.ranges) {}
        
        Markov& Markov::operator=(const Markov &obj) {
            transitions = obj.transitions;
            currentState = obj.currentState;
            ranges = obj.ranges;
            return (*this);
        }

        // Calcula o próximo estado da cadeia de markov
        void Markov::next_state() {
            int32_t randomNumber = probabilityGenerator(seed);
            // Calcula o offset do vetor baseado no estado
            int32_t offset = currentState * 5;

            // possibilities corresponde à chance da cadeia cair em um determinado estado
            array<float, 5> possibilities { { 0.0,0.0,0.0,0.0,0.0 }};
            
            // Inicializando possibilities
            for(int32_t i = 0; i < 5; i++) {
                for(int32_t j = 0; j <= i; j++) {
                    possibilities[i] += transitions[offset + j];     
                }
            }
            
            if(randomNumber <= possibilities[0]) {
                currentState = 0;                        
            }    
            else if(randomNumber <= possibilities[1]) {
                currentState = 1;             
            }    
            else if(randomNumber <= possibilities[2]) {
                currentState = 2;
            }    
            else if(randomNumber <= possibilities[3]) {
                currentState = 3;        
            }    
            else if(randomNumber <= possibilities[4]) {
                currentState = 4;
            }
        }

        // Calcula um valor baseado no intervalo do estado atual
        double Markov::calculate_state() {
            Range this_range = ranges[currentState];
            // Cria um número aleatório baseado no range
            std::uniform_real_distribution<double> value_generator(this_range.getLowerBound(), this_range.getUpperBound());
            double val = value_generator(seed);
            return val;
        }
        
        const string Markov::toString() const {
            stringstream sstr;

            sstr << "Markov" << "" << endl;

            return sstr.str();
        }

    }
}
