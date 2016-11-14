#ifndef SIMPLESTRING_H
#define SIMPLESTRING_H
#include <string.h>
class SimpleString 
{
	public:
		SimpleString() { strPtr = 0; };
		SimpleString(const char *str) { strPtr = 0; setString(str); };
		SimpleString(const SimpleString &right) {
			strPtr = 0;
			setString(right.getString());
		};
		SimpleString &operator=(const SimpleString &right) {
			if (this != &right) {
				strPtr = 0;
				setString(right.getString());
			}
			return *this;
		};
		~SimpleString() { if (strPtr != 0) { delete [] strPtr; } };
		const char * getString() const { return strPtr; };
		void setString(const char *string) {
			if (strPtr != 0)
				delete [] strPtr;
			int length = (int)strlen(string);
			strPtr = new char[length+1];
			strcpy(strPtr, string);
		};
	private:
		char *strPtr;
};
#endif
