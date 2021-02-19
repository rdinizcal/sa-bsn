#ifndef SENSORCONFIGURATION_HPP
#define SENSORCONFIGURATION_HPP

#include <string>
#include <math.h>
#include <iostream>
#include <array>
#include <sstream>

#include "libbsn/range/Range.hpp"

namespace bsn {
	namespace configuration {

		class SensorConfiguration {
			public:
				SensorConfiguration();

				SensorConfiguration(int32_t i, bsn::range::Range,
									std::array<bsn::range::Range, 2>,
									std::array<bsn::range::Range, 2>,
									std::array<bsn::range::Range, 3>);

				SensorConfiguration(const SensorConfiguration & /*obj*/);
				SensorConfiguration &operator=(const SensorConfiguration & /*obj*/);

			  private:
				
				int32_t id;
				bsn::range::Range lowRisk;	
				std::array<bsn::range::Range, 2> mediumRisk;
				std::array<bsn::range::Range, 2> highRisk;
				bsn::range::Range lowPercentage, midPercentage, highPercentage;

			public:

				// Retorna o estado de risco a partir dos intervalos
				double evaluateNumber(double number);

				// Retorna o quão deslocado do meio um valor está
				double getDisplacement(bsn::range::Range, double, std::string );

				// Converte para o percentual final( {var}_percentage )
				double convertRealPercentage(bsn::range::Range range, double number);

				// Verifica se valor passado é de baixo risco
				bool isLowRisk(double/*val*/);

				// Verifica se valor passado é de médio risco
				bool isMediumRisk(double/*val*/);

				// Verifica se valor passado é de alto risco
				bool isHighRisk(double/*val*/);

				int32_t getId() const;
				void setId(const int32_t);

				bsn::range::Range getLowRisk() const;
				void setLowRisk(const bsn::range::Range);
				
				std::array<bsn::range::Range, 2> getMediumRisk() const;
				void setMediumRisk(const std::array<bsn::range::Range, 2>);

				std::array<bsn::range::Range, 2> getHighRisk() const;
				void setHighRisk(const std::array<bsn::range::Range, 2>);

				bsn::range::Range getLowPercentage() const;
				void setLowPercentage(const bsn::range::Range);

				bsn::range::Range getMidPercentage() const;
				void setMidPercentage(const bsn::range::Range);

				bsn::range::Range getHighPercentage() const;
				void setHighPercentage(const bsn::range::Range);

				const std::string toString() ;
		};


	}
}

#endif