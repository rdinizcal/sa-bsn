#include "bsn/processor/Processor.hpp"

using namespace std;

namespace bsn {
    namespace processor {
        // Retorna o id baseado no tipo
        int32_t get_sensor_id(string type) {
            if (type == "thermometer")
                return 0;
            else if (type == "ecg")
                return 1;
            else if (type == "oximeter")
                return 2;
            else if (type == "bpms")
                return 3;
            else if (type == "bpmd")		
                return 4;
            else {
                cout << "UNKNOWN TYPE " + type + '\n';
                return -1;
            }

        }

        double get_value(string packet){
            double ret = stod(packet.substr(packet.find('-')+1,packet.length()));
            return ret;
        }

        // Retorna true se as tres listas são não vazias
        bool available_to_process(vector<list<double>> packetsReceived){
            bool available = false;

            for(auto packet_list : packetsReceived){
                if(!packet_list.empty()){
                    available = true;
                    break;
                }
            }

            return available;
        }

        double data_fuse(vector<list<double>> &packetsReceived) {	
            double average, risk_status;
            int32_t count = 0;
            average = 0;

            // Se não existiver disponível não processa
            if(!available_to_process(packetsReceived))
                return -1;
            
            for(auto &packet_list : packetsReceived){
                if(!packet_list.empty()) {
                    // Soma à média e retira da fila
                    average += packet_list.front();
                    
                    // Descarta o pacote processado se existem
                    // Mais outros para serem processados
                    if(packet_list.size() > 1) {
                        packet_list.pop_front();
                    }			
                    count++;
                }
            }	
            // Calcula a media partir da soma dividida pelo número de pacotes lidos
            risk_status = (average / count);
            // Status de risco do paciente dado em porcentagem

            // 85.0 é um número totalmente ARBITRARIO
            if(risk_status > 66.0){
                cout << "============ EMERGENCY ============(" << risk_status << '%' << ")" << endl;
            }
            else{
                cout << "General risk status: " << risk_status << '%' << endl;
            }

            return risk_status;
        }
    }
}