#ifndef __MEASUREMENT_H__
#define __MEASUREMENT_H__

class measurement {
	public:
		virtual void* getMeasurement(string name);
		virtual std::list<string> getFields() = 0;
}
