#ifndef __MEASUREMENT_H__
#define __MEASUREMENT_H__

#include <string>
#include <list>

class measurement {
	public:
		virtual void* getMeasurement(std::string) = 0;
		virtual std::list<std::string> getFields() = 0;
};

#endif
