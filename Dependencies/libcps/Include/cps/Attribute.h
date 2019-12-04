/** Attributes
 *
 * @file
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2018, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#pragma once
#include <iostream>

#include <cps/Definitions.h>
#include <cps/PtrFactory.h>
#include <cps/MathUtils.h>
#include <cps/Config.h>

#ifdef WITH_PYTHON
#ifdef _DEBUG
  #undef _DEBUG
  #include <Python.h>
  #define _DEBUG
#else
  #include <Python.h>
#endif
#endif

#ifdef WITH_NUMPY
  #define NO_IMPORT_ARRAY
  #define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
  #include <numpy/arrayobject.h>
#endif

namespace CPS {

namespace Flags {
	// TODO clarify access rules for within simulation
	enum Access : int {
		read = 1,
		write = 2,
		setter = 4,
		getter = 8,
		owner = 16,
		state = 32, /// This attribute constitutes a state of the component which should be resetted between simulation runs
	};
}

	class AttributeBase :
		public std::enable_shared_from_this<AttributeBase> {

	protected:
		/// Flag to determine access rules for this attribute.
		int mFlags;
		/// Pointer that serves as an unique identifier to this attribute.
		std::shared_ptr<AttributeBase> mRefAttribute;

		AttributeBase(int flags) :
			mFlags(flags)
		{ };

		AttributeBase(int flags, std::shared_ptr<AttributeBase> refAttribute) :
			mFlags(flags), mRefAttribute(refAttribute)
		{ };

		virtual ~AttributeBase() { }

	public:
		typedef std::shared_ptr<AttributeBase> Ptr;
		typedef std::vector<Ptr> List;
		typedef std::map<String, Ptr> Map;

		virtual String toString() const = 0;

		int flags() const {
			return mFlags;
		}

		virtual void reset() = 0;

		static AttributeBase::Ptr getRefAttribute(AttributeBase::Ptr& attr) {
			AttributeBase::Ptr& p = attr;
			while (p && p->mRefAttribute)
				p = p->mRefAttribute;
			return p;
		}

#ifdef WITH_PYTHON
		virtual void fromPyObject(PyObject *po) = 0;
		virtual PyObject * toPyObject() = 0;
#ifdef WITH_NUMPY
		virtual PyArray_Descr * toPyArrayDescr() { return nullptr; }
		virtual PyObject * toPyArray() { return nullptr; }
#endif
#endif
	};

	template<class T>
	class Attribute :
		public AttributeBase,
		public SharedFactory<Attribute<T>> {
		friend class SharedFactory<Attribute<T>>;

	public:
		using Getter = std::function<T()>;
		using Setter = std::function<void(const T&)>;

	protected:
		T *mValue;

		Setter mSetter;
		Getter mGetter;

	public:
		typedef T Type;
		typedef std::shared_ptr<Attribute<T>> Ptr;

		Attribute(int flags = Flags::read) :
			AttributeBase(flags | Flags::owner) {
			mValue = new T;
		}

		Attribute(T *v, int flags = Flags::read, AttributeBase::Ptr refAttribute = AttributeBase::Ptr()) :
			AttributeBase(flags, refAttribute),
			mValue(v)
		{ };

		Attribute(Setter set = Setter(), Getter get = Getter(), int flags = Flags::read, AttributeBase::Ptr refAttribute = AttributeBase::Ptr()) :
			AttributeBase(flags | Flags::setter | Flags::getter, refAttribute),
			mSetter(set),
			mGetter(get)
		{ };

		Attribute(Getter get = Getter(), int flags = Flags::read, AttributeBase::Ptr refAttribute = AttributeBase::Ptr()) :
			AttributeBase(flags | Flags::getter, refAttribute),
			mGetter(get)
		{ };

		virtual ~Attribute() {
			if (mFlags & Flags::owner)
				delete mValue;
		}

		void set(const T &v) {
			// Check access
			if (mFlags & Flags::write) {
				if (mFlags & Flags::setter)
					mSetter(v);
				else
					*mValue = v;
			}
			else
				throw AccessException();
		}

		const T& get() const {
			// Check access
			if (mFlags & Flags::read) {
				if (mFlags & Flags::getter) {
					// TODO: this can be done nicer..
					std::cerr << "ERROR: do not use get by reference for getter attributes." << std::endl;
					throw AccessException();
				}
				else {
					return *mValue;
				}
			}
			else
				throw AccessException();
		}

		void reset() {
			// TODO: we might want to provide a default value via the constructor
			T resetValue = T();

			// Only states are resetted!
			if (mFlags & Flags::state)
				set(resetValue);
		}

		T getByValue() const {
			// Check access
			if (mFlags & Flags::read) {
				if (mFlags & Flags::getter)
					return mGetter();
				else
					return *mValue;
			}
			else
				throw AccessException();
		}

		String toString() const {
			return std::to_string(this->getByValue());
		}

		/// @brief User-defined cast operator
		///
		/// Allows attributes to be casted to their value type:
		///
		/// Real v = 1.2;
		/// auto a = Attribute<Real>(&v);
		///
		/// Real x = v;
		///
		operator T() {
			return get();
		}

		operator T&() {
			return *mValue;
		}

		/// @brief User-defined assignment operator
		///
		/// Real v = 1.2;
		/// auto a = Attribute<Real>(&v);
		///
		/// a = 3;
		///
		Attribute<T>& operator=(const T &other) {
			set(other);
			return *this;
		}

#ifdef WITH_PYTHON
		virtual void fromPyObject(PyObject *po);
		virtual PyObject * toPyObject();
#ifdef WITH_NUMPY
		virtual PyArray_Descr * toPyArrayDescr() { return nullptr; }
		virtual PyObject * toPyArray() { return nullptr; }
#endif
#endif
	};

	class ComplexAttribute :
		public Attribute<Complex> {
	public:
		typedef std::shared_ptr<ComplexAttribute> Ptr;

		ComplexAttribute(Complex *v, int flags = Flags::read,
			AttributeBase::Ptr refAttribute = AttributeBase::Ptr()) :
			Attribute<Complex>(v, flags, refAttribute) { };

		ComplexAttribute(Getter get = Getter(), int flags = Flags::read,
			AttributeBase::Ptr refAttribute = AttributeBase::Ptr()) :
			Attribute<Complex>(get, flags, refAttribute) { };

		// From the C++ standard:
		// For any pointer to an element of an array of complex<T> named p and any valid array index i,
		// reinterpret_cast<T*>(p)[2*i] is the real part of the complex number p[i], and
		// reinterpret_cast<T*>(p)[2*i + 1] is the imaginary part of the complex number p[i]

		Attribute<Real>::Ptr real() {
			Attribute<Real>::Getter get = [this]() -> Real {
				return this->getByValue().real();
			};
			Attribute<Real>::Setter set = [this](Real realPart) -> void {
				Complex copyValue = this->getByValue();
				this->set(Complex(realPart, copyValue.imag()));
			};
			return Attribute<Real>::make(nullptr, get, mFlags, shared_from_this());
			//Real *realPart = &reinterpret_cast<Real*>(mValue)[0];
			//return Attribute<Real>::make(realPart, mFlags, shared_from_this());
		}

		Attribute<Real>::Ptr imag() {
			Attribute<Real>::Getter get = [this]() -> Real {
				return this->getByValue().imag();
			};
			Attribute<Real>::Setter set = [this](Real imagPart) -> void {
				Complex copyValue = this->getByValue();
				this->set(Complex(copyValue.real(), imagPart));
			};
			return Attribute<Real>::make(nullptr, get, mFlags, shared_from_this());
			//Real *imagPart = &reinterpret_cast<Real*>(mValue)[1];
			//return Attribute<Real>::make(imagPart, mFlags, shared_from_this());
		}

		Attribute<Real>::Ptr mag() {
			Attribute<Real>::Getter get = [this]() -> Real {
				return Math::abs(this->getByValue());
			};
			Attribute<Real>::Setter set = [this](Real r) -> void {
				Complex z = this->getByValue();
				this->set(Math::polar(r, Math::phase(z)));
			};
			return Attribute<Real>::make(set, get, mFlags, shared_from_this());
		}

		Attribute<Real>::Ptr phase(Bool isRad = true) {
			Attribute<Real>::Getter get = [this, isRad]() -> Real {
				return isRad ? Math::phase(this->getByValue())
				             : Math::phaseDeg(this->getByValue());
			};
			Attribute<Real>::Setter set = [this, isRad](Real p) -> void {
				Complex z = this->getByValue();
				this->set(isRad ? Math::polar(std::abs(z), p)
				                : Math::polarDeg(std::abs(z), p));
			};
			return Attribute<Real>::make(set, get, mFlags, shared_from_this());
		}

		ComplexAttribute::Ptr scale(Complex factor) {
			ComplexAttribute::Getter get = [this, factor]() -> Complex {
				return factor*this->getByValue();
			};
			return std::make_shared<ComplexAttribute>(get, mFlags, shared_from_this());
		}
	};

	template<typename T>
	class MatrixAttribute : public Attribute<MatrixVar<T>> {
	protected:
		using Index = typename MatrixVar<T>::Index;
		using Attribute<MatrixVar<T>>::mFlags;
		using Attribute<MatrixVar<T>>::mValue;
		using std::enable_shared_from_this<AttributeBase>::shared_from_this;
	public:
		typedef std::shared_ptr<MatrixAttribute> Ptr;

		typename Attribute<T>::Ptr coeff(Index row, Index col) {
			typename Attribute<T>::Getter get = [this, row, col]() -> T {
				return this->getByValue()(row, col);
			};
			//typename Attribute<T>::Setter set = [](T n) -> void {
			//	MatrixVar<T> &mat = this->getByValue();
			//	mat(row, col) = n;
			//	this->set(mat);
			//};
			return Attribute<T>::make(get, mFlags, shared_from_this());
			//T *ptr = &mValue->data()[mValue->cols() * row + col]; // Column major
			//return Attribute<T>::make(ptr, mFlags, shared_from_this());
		}
	};

	class MatrixRealAttribute : public Attribute<Matrix> {
	protected:
		using Index = typename Matrix::Index;
		using Attribute<Matrix>::mFlags;
		using Attribute<Matrix>::mValue;
		using std::enable_shared_from_this<AttributeBase>::shared_from_this;
	public:
		typedef std::shared_ptr<MatrixRealAttribute> Ptr;

		typename Attribute<Real>::Ptr coeff(Index row, Index col) {
			typename Attribute<Real>::Getter get = [this, row, col]() -> Real {
				return this->getByValue()(row, col);
			};
			//typename Attribute<T>::Setter set = [](T n) -> void {
			//	Matrix &mat = this->get();
			//	mat(row, col) = n;
			//	this->set(mat);
			//};
			return Attribute<Real>::make(get, mFlags, shared_from_this());
			//T *ptr = &mValue->data()[mValue->cols() * row + col]; // Column major
			//return Attribute<T>::make(ptr, mFlags, shared_from_this());
		}
	};

	class MatrixCompAttribute : public Attribute<MatrixComp> {
	protected:
		using Index = typename MatrixComp::Index;
		using Attribute<MatrixComp>::mFlags;
		using Attribute<MatrixComp>::mValue;
		using std::enable_shared_from_this<AttributeBase>::shared_from_this;
	public:
		typedef std::shared_ptr<MatrixCompAttribute> Ptr;

		ComplexAttribute::Ptr coeff(Index row, Index col) {
			ComplexAttribute::Getter get = [this, row, col]() -> Complex {
				return this->getByValue()(row, col);
			};
			return std::make_shared<ComplexAttribute>(get, mFlags, shared_from_this());
			//Complex *ptr = &mValue->data()[mValue->cols() * row + col]; // Column major
			//return std::make_shared<ComplexAttribute>(ptr, mFlags, shared_from_this());
		}

		Attribute<Real>::Ptr coeffReal(Index row, Index col) {
			Attribute<Real>::Getter get = [this, row, col]() -> Real {
				return this->getByValue()(row,col).real();
			};
			return Attribute<Real>::make(get, mFlags, shared_from_this());
			//Complex *ptr = &mValue->data()[mValue->cols() * row + col]; // Column major
			//Real *realPart = &reinterpret_cast<Real*>(ptr)[0];
			//return Attribute<Real>::make(&realPart, mFlags, shared_from_this());
		}

		Attribute<Real>::Ptr coeffImag(Index row, Index col) {
			Attribute<Real>::Getter get = [this, row, col]() -> Real {
				return this->getByValue()(row,col).imag();;
			};
			return Attribute<Real>::make(get, mFlags, shared_from_this());
		}

		Attribute<Real>::Ptr coeffMag(Index row, Index col) {
			Attribute<Real>::Getter get = [this, row, col]() -> Real {
				return Math::abs(this->get()(row,col));
			};
			return Attribute<Real>::make(get, mFlags, shared_from_this());
			//Complex *ptr = &mValue->data()[mValue->cols() * row + col]; // Column major
			//Real *realPart = &reinterpret_cast<Real*>(ptr)[0];
			//return Attribute<Real>::make(&realPart, mFlags, shared_from_this());
		}

		Attribute<Real>::Ptr coeffPhase(Index row, Index col) {
			Attribute<Real>::Getter get = [this, row, col]() -> Real {
				return Math::phase(this->get()(row,col));
			};
			return Attribute<Real>::make(get, mFlags, shared_from_this());
		}

	};

	template<>
	String Attribute<Complex>::toString() const;

	template<>
	String Attribute<String>::toString() const;

	template<>
	String Attribute<MatrixComp>::toString() const;

	template<>
	String Attribute<Matrix>::toString() const;
}
