#include "libbsn/generator/DataGenerator.hpp"
#include "libbsn/range/Range.hpp"

namespace bsn {
    namespace generator {
        DataGenerator::DataGenerator() : markovChain(), seed() {}

        DataGenerator::DataGenerator(const Markov& markov) : 
            markovChain(markov),
            seed() {}

        DataGenerator::DataGenerator(const DataGenerator& obj) :
            markovChain(obj.markovChain),
            seed() {}

        DataGenerator::~DataGenerator() {}

        DataGenerator& DataGenerator::operator=(const DataGenerator& obj) {
            markovChain = obj.markovChain;
            return (*this);
        }

        std::uniform_int_distribution<int> probabilityGenerator(1,100);


        void DataGenerator::setSeed() {
            std::random_device rd;
            std::mt19937 aux(rd());
            seed = aux;
        }

        void DataGenerator::nextState() {
            int32_t randomNumber = probabilityGenerator(seed);
            // Calcula o offset do vetor baseado no estado
            int32_t offset = markovChain.currentState * 5;
            
            if (randomNumber <= markovChain.transitions[offset]) {
                markovChain.currentState = 0;                        
            }    
            else if (randomNumber <= markovChain.transitions[offset + 1]) {
                markovChain.currentState = 1;             
            }    
            else if (randomNumber <= markovChain.transitions[offset + 2]) {
                markovChain.currentState = 2;
            }     
            else if (randomNumber <= markovChain.transitions[offset + 3]) {
                markovChain.currentState = 3;        
            }    
            else if (randomNumber <= markovChain.transitions[offset + 4]) {
                markovChain.currentState = 4;
            }
        }

        double DataGenerator::calculateValue() {
            if (markovChain.currentState > 4 || markovChain.currentState < 0){
                throw std::out_of_range("current state is out of bounds");
            }

            bsn::range::Range range = markovChain.states[markovChain.currentState];
            // Cria um número aleatório baseado no range
            std::uniform_real_distribution<double> value_generator(range.getLowerBound(), range.getUpperBound());
            double val = value_generator(seed);
            return val;
        }

        double DataGenerator::getValue() {
            return calculateValue();
        }
    }
    
} // namespace bs 


