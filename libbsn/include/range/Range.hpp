#ifndef RANGE_HPP
#define RANGE_HPP

#include <string>
#include <stdexcept>
#include <iostream>
#include <stdint.h>
#include <sstream>

namespace bsn {
	namespace range {

		class Range {	
					
			public:
				Range();
				Range(const double &/*lower bound*/, const double &/*upper bound*/);
				~Range();

				Range(const Range & /*obj*/);
				Range &operator=(const Range & /*obj*/);

				// Verifica se um elemento esta dentro do intervalo estipulado
				bool in_range(double element);
				double convert(double new_lb, double new_ub, double input);
				std::string to_print();

				void setLowerBound (const double &/*_lowerBound*/);
				double getLowerBound () const;

				void setUpperBound (const double &/*_upperBound*/);
				double getUpperBound () const;

				const std::string toString() const;

			private:
				double lowerBound;
				double upperBound;

		};

	}
}

#endif 