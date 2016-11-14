#ifndef VECTORJI_H
#define VECTORJI_H

#include <vector>
using std::vector;

template <class T>

class VectorJI
{
	public:
		VectorJI() { vec = new vector<T>(); };
		VectorJI(const VectorJI<T> &right) {
			vec = new vector<T>();
			vec->assign(right.vec->begin(), right.vec->end());
		};
		VectorJI<T> &operator=(const VectorJI<T> &right) {
			if (this != &right) {
				delete vec;
				vec = new vector<T>();
				vec->assign(right.vec->begin(), right.vec->end());
			}
			return *this;
		};
		VectorJI(int size) { vec = new vector<T>(size); };
		~VectorJI() { delete vec; };
		void add(const T& data) { vec->push_back(data); };
		void add(int arg0, const T& data) { vec->insert(vec->begin()+arg0, data); };
		void ensureCapacity(int arg0) { vec->reserve(arg0); } ;
		void setSize(int size) { vec->resize(size); };
		void setElementAt(const T& data, int arg0) { remove(arg0); add(arg0, data); };
		T remove(int element) { 
			T old = vec->at(element); 
			vec->erase(vec->begin()+element);
			return old;
		};
		T& elementAt(int i) const { return vec->at(i); };
		T& get(int i) const { return elementAt(i); };
		int size() const { return (int)vec->size(); };
		bool isEmpty() const { return vec->empty(); };
		bool isFull() const { return (vec->size() == vec->capacity()); };
		void trimToSize() {; };
		void destroy() { vec->clear(); };
	private:
		vector<T> *vec;
};


#endif