/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once
#include <iostream>

#include <cps/Definitions.h>
#include <cps/PtrFactory.h>
#include <cps/MathUtils.h>
#include <cps/Config.h>
namespace CPS {

namespace Flags {
	// TODO clarify access rules for within simulation
	enum Access : int {
		read = 1,
		write = 2,
		state = 32, /// This attribute constitutes a state of the component which should be resetted between simulation runs
	};
}

	// template<class T, class U> 
	// class DerivedAttribute;
	class AttributeBase :
		public std::enable_shared_from_this<AttributeBase> {

	protected:
		/// Flag to determine access rules for this attribute.
		int mFlags;

		AttributeBase(int flags) :
			mFlags(flags)
		{ };

	public:
		typedef std::shared_ptr<AttributeBase> Ptr;
		typedef std::vector<Ptr> List;
		typedef std::map<String, Ptr> Map;

		// //TODO: Delete
		// enum class Modifier { real, imag, mag, phase };

		virtual String toString() const = 0;

		int flags() const {
			return mFlags;
		}

		virtual void reset() = 0;

		// //TODO: Delete
		// static AttributeBase::Ptr getRefAttribute(AttributeBase::Ptr &attr) {
		// 	return attr;
		// }
	};

	template<class T>
	class Attribute :
		public AttributeBase,
		public SharedFactory<Attribute<T>>,
		public std::enable_shared_from_this<Attribute<T>> {
		friend class SharedFactory<Attribute<T>>;

	protected:
		//FIXME: When the value is actually an external reference (set by the second constructor), destroying this shared ptr will crash the program.
		//The goal here should be to eliminate all uses of this second constructor,
		//storing the attributes themselves as class members instead of references to the underlying data
		std::shared_ptr<T> mData;

	public:
		typedef T Type;
		typedef std::shared_ptr<Attribute<T>> Ptr;

		Attribute(int flags = Flags::read) :
			AttributeBase(flags), mData(std::make_shared<T>()) { }

		// // Delete
		// Attribute(T *v, int flags = Flags::read, const AttributeBase::Ptr &refAttribute = AttributeBase::Ptr()) :
		// 	Attribute(flags)
		// { };

		virtual void set(T& value) = 0;

		virtual const T& get() = 0;

		virtual void reset() {
			// TODO: we might want to provide a default value via the constructor
			T resetValue = T();

			// Only states are resetted!
			if (mFlags & Flags::state)
				set(resetValue);
		}

		// (Maybe delete)
		// virtual T getByValue() const {
		// 	return *get();
		// }

		String toString() {
			return std::to_string(get());
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
		operator const T&() {
			return this->get();
		}

		/// @brief User-defined dereference operator
		///
		/// Allows easier access to the attribute's underlying data
		const T& operator*(){
			return this->get();
		}

		// /// Do not use!
		// /// Only used for Eigen Matrix - Sundials N_Vector interfacing in N_VSetArrayPointer
		// operator T&() {
		// 	return this->get()->get();
		// }

		/// @brief User-defined assignment operator
		///
		/// Real v = 1.2;
		/// auto a = Attribute<Real>(&v);
		///
		/// a = 3;
		///
		Attribute<T>& operator=(T other) {
			set(other);
			return *this;
		}

		Attribute<T>& operator=(T& other) {
			set(other);
			return *this;
		}

		// template <class U>
		// typename Attribute<U>::Ptr derive(
		// 	int flags,
		// 	typename DerivedAttribute<U, T>::Setter set = DerivedAttribute<U, T>::Setter(),
		// 	typename DerivedAttribute<U, T>::Getter get = DerivedAttribute<U, T>::Getter()
		// )
		// {
		// 	return std::make_shared<DerivedAttribute<U,T>>(shared_from_this(), set, get, flags);
		// }

		// template <class U>
		// typename Attribute<U>::Ptr derive(
		// 	typename DerivedAttribute<U, T>::Setter set = DerivedAttribute<U, T>::Setter(),
		// 	typename DerivedAttribute<U, T>::Getter get = DerivedAttribute<U, T>::Getter()
		// )
		// {
		// 	return derive<U>(set, get, this->mFlags);
		// }
	};

	template<class DependentType, class... DependencyTypes>
	class AttributeUpdateTask :
		public SharedFactory<AttributeUpdateTask<DependentType, DependencyTypes...>> {

	public:
		enum UpdateTaskKind {
			UPDATE_ONCE,
			UPDATE_ON_GET,
			UPDATE_ON_SET,
			UPDATE_ON_SIMULATION_STEP,
		};
	protected:
		using Actor = std::function<bool(std::shared_ptr<DependentType>&, std::shared_ptr<Attribute<DependencyTypes>>...)>;
		std::tuple<std::shared_ptr<Attribute<DependencyTypes>>...> mDependencies;
		Actor mActorFunction;
		UpdateTaskKind mKind;

	public:
		AttributeUpdateTask(UpdateTaskKind kind, Actor actorFunction, std::shared_ptr<Attribute<DependencyTypes>>... dependencies)
			: mKind(kind), mActorFunction(actorFunction), mDependencies(std::forward<std::shared_ptr<Attribute<DependencyTypes>>>(dependencies)...) {}

		bool executeUpdate(std::shared_ptr<DependentType> &dependent) {
			return actorFunction(dependent, mDependencies);
		}
	};


	// ///T: Type of the derived attribute
	// ///U: Type of the attribute this attribute is derived from
	// template<class T, class U> 
	// class DerivedAttribute : public Attribute<T> {

	// public:
	// 	using Getter = std::function<const T&(std::shared_ptr<Attribute<U>>)>;
	// 	using Setter = std::function<void(std::shared_ptr<Attribute<U>>, const T&)>;

	// protected:
	// 	/// The attribute this attribute is derived from
	// 	std::shared_ptr<Attribute<U>> mParent;

	// 	/// Setter function for this derived attribute. Might be empty to allow direct write access to the parent attribute
	// 	Setter mSetter;
	// 	/// Getter function for this derived attribute. Might be empty to allow direct read access to the parent attribute
	// 	Getter mGetter;

	// 	/// Constructor for a derived attribute. Should only be used in another attribute's `derive`-method
	// 	DerivedAttribute(Attribute<U> &parent, Setter set = Setter(), Getter get = Getter(), int flags = Flags::read) :
	// 		Attribute<T>(flags) {
	// 			if (!get && (typeid(T) != typeid(U))) {
	// 				throw TypeException("Tried to derive an argument with a type differing from the parent argument, but no Getter function was provided!");
	// 			}
	// 			if (!set && (typeid(T) != typeid(U))) {
	// 				throw TypeException("Tried to derive an argument with a type differing from the parent argument, but no Setter function was provided!");
	// 			}
	// 		}

	// public:
	// 	virtual void set(const T &v) override {
	// 		if (AttributeBase::mFlags & Flags::write) {
	// 			if (mSetter) {
	// 				mSetter(mParent, v);
	// 			} else {
	// 				mParent->set(dynamic_cast<const U&>(v));
	// 			}
	// 		}
	// 		else
	// 			throw AccessException();
	// 	}

	// 	virtual const T& get() const override {
	// 		if (AttributeBase::mFlags & Flags::read) {
	// 			if (mGetter) {
	// 				return mGetter(mParent);
	// 			} else {
	// 				return dynamic_cast<const T&>(mParent->get());
	// 			}
	// 		}
	// 		else
	// 			throw AccessException();
	// 	}

	// 	virtual void reset() {
	// 		// TODO: we might want to provide a default value via the constructor
	// 		T resetValue = T();

	// 		// Only states are resetted!
	// 		if (AttributeBase::mFlags & Flags::state)
	// 			set(resetValue);
	// 	}
	// };


	// // Replace by DerivedAttribute
	// class ComplexAttribute :
	// 	public Attribute<Complex> {
	// public:
	// 	typedef std::shared_ptr<ComplexAttribute> Ptr;

	// 	ComplexAttribute(Complex *v, int flags = Flags::read,
	// 		const AttributeBase::Ptr &refAttribute = AttributeBase::Ptr()) :
	// 		Attribute<Complex>(v, flags, refAttribute) { };

	// 	ComplexAttribute(Getter get = Getter(), int flags = Flags::read,
	// 		const AttributeBase::Ptr &refAttribute = AttributeBase::Ptr()) :
	// 		Attribute<Complex>(get, flags, refAttribute) { };

	// 	// From the C++ standard:
	// 	// For any pointer to an element of an array of complex<T> named p and any valid array index i,
	// 	// reinterpret_cast<T*>(p)[2*i] is the real part of the complex number p[i], and
	// 	// reinterpret_cast<T*>(p)[2*i + 1] is the imaginary part of the complex number p[i]

	// 	Attribute<Real>::Ptr real() {
	// 		Attribute<Real>::Getter get = [this]() -> Real {
	// 			return this->getByValue().real();
	// 		};
	// 		Attribute<Real>::Setter set = [this](Real realPart) -> void {
	// 			Complex copyValue = this->getByValue();
	// 			this->set(Complex(realPart, copyValue.imag()));
	// 		};
	// 		return Attribute<Real>::make(nullptr, get, mFlags, shared_from_this());
	// 		//Real *realPart = &reinterpret_cast<Real*>(mData)[0];
	// 		//return Attribute<Real>::make(realPart, mFlags, shared_from_this());
	// 	}

	// 	Attribute<Real>::Ptr imag() {
	// 		Attribute<Real>::Getter get = [this]() -> Real {
	// 			return this->getByValue().imag();
	// 		};
	// 		Attribute<Real>::Setter set = [this](Real imagPart) -> void {
	// 			Complex copyValue = this->getByValue();
	// 			this->set(Complex(copyValue.real(), imagPart));
	// 		};
	// 		return Attribute<Real>::make(nullptr, get, mFlags, shared_from_this());
	// 		//Real *imagPart = &reinterpret_cast<Real*>(mData)[1];
	// 		//return Attribute<Real>::make(imagPart, mFlags, shared_from_this());
	// 	}

	// 	Attribute<Real>::Ptr mag() {
	// 		Attribute<Real>::Getter get = [this]() -> Real {
	// 			return Math::abs(this->getByValue());
	// 		};
	// 		Attribute<Real>::Setter set = [this](Real r) -> void {
	// 			Complex z = this->getByValue();
	// 			this->set(Math::polar(r, Math::phase(z)));
	// 		};
	// 		return Attribute<Real>::make(set, get, mFlags, shared_from_this());
	// 	}

	// 	Attribute<Real>::Ptr phase(Bool isRad = true) {
	// 		Attribute<Real>::Getter get = [this, isRad]() -> Real {
	// 			return isRad ? Math::phase(this->getByValue())
	// 			             : Math::phaseDeg(this->getByValue());
	// 		};
	// 		Attribute<Real>::Setter set = [this, isRad](Real p) -> void {
	// 			Complex z = this->getByValue();
	// 			this->set(isRad ? Math::polar(std::abs(z), p)
	// 			                : Math::polarDeg(std::abs(z), p));
	// 		};
	// 		return Attribute<Real>::make(set, get, mFlags, shared_from_this());
	// 	}

	// 	ComplexAttribute::Ptr scale(Complex factor) {
	// 		ComplexAttribute::Getter get = [this, factor]() -> Complex {
	// 			return factor*this->getByValue();
	// 		};
	// 		return std::make_shared<ComplexAttribute>(get, mFlags, shared_from_this());
	// 	}
	// };

	// // Replace by DerivedAttribute
	// template<typename T>
	// class MatrixAttribute : public Attribute<MatrixVar<T>> {
	// protected:
	// 	using Index = typename MatrixVar<T>::Index;
	// 	using Attribute<MatrixVar<T>>::mFlags;
	// 	using Attribute<MatrixVar<T>>::mData;
	// 	using std::enable_shared_from_this<AttributeBase>::shared_from_this;
	// public:
	// 	typedef std::shared_ptr<MatrixAttribute> Ptr;

	// 	typename Attribute<T>::Ptr coeff(Index row, Index col) {
	// 		typename Attribute<T>::Getter get = [this, row, col]() -> T {
	// 			return this->getByValue()(row, col);
	// 		};
	// 		//typename Attribute<T>::Setter set = [](T n) -> void {
	// 		//	MatrixVar<T> &mat = this->getByValue();
	// 		//	mat(row, col) = n;
	// 		//	this->set(mat);
	// 		//};
	// 		return Attribute<T>::make(get, mFlags, shared_from_this());
	// 		//T *ptr = &mData->data()[mData->cols() * row + col]; // Column major
	// 		//return Attribute<T>::make(ptr, mFlags, shared_from_this());
	// 	}
	// };

	// // Replace by DerivedAttribute
	// class MatrixRealAttribute : public Attribute<Matrix> {
	// protected:
	// 	using Index = typename Matrix::Index;
	// 	using Attribute<Matrix>::mFlags;
	// 	using Attribute<Matrix>::mData;
	// 	using std::enable_shared_from_this<AttributeBase>::shared_from_this;
	// public:
	// 	typedef std::shared_ptr<MatrixRealAttribute> Ptr;

	// 	typename Attribute<Real>::Ptr coeff(Index row, Index col) {
	// 		typename Attribute<Real>::Getter get = [this, row, col]() -> Real {
	// 			return this->getByValue()(row, col);
	// 		};
	// 		//typename Attribute<T>::Setter set = [](T n) -> void {
	// 		//	Matrix &mat = this->get();
	// 		//	mat(row, col) = n;
	// 		//	this->set(mat);
	// 		//};
	// 		return Attribute<Real>::make(get, mFlags, shared_from_this());
	// 		//T *ptr = &mData->data()[mData->cols() * row + col]; // Column major
	// 		//return Attribute<T>::make(ptr, mFlags, shared_from_this());
	// 	}
	// };

	// // Replace by DerivedAttribute
	// class MatrixCompAttribute : public Attribute<MatrixComp> {
	// protected:
	// 	using Index = typename MatrixComp::Index;
	// 	using Attribute<MatrixComp>::mFlags;
	// 	using Attribute<MatrixComp>::mData;
	// 	using std::enable_shared_from_this<AttributeBase>::shared_from_this;
	// public:
	// 	typedef std::shared_ptr<MatrixCompAttribute> Ptr;

	// 	ComplexAttribute::Ptr coeff(Index row, Index col) {
	// 		ComplexAttribute::Getter get = [this, row, col]() -> Complex {
	// 			return this->getByValue()(row, col);
	// 		};
	// 		return std::make_shared<ComplexAttribute>(get, mFlags, shared_from_this());
	// 		//Complex *ptr = &mData->data()[mData->cols() * row + col]; // Column major
	// 		//return std::make_shared<ComplexAttribute>(ptr, mFlags, shared_from_this());
	// 	}

	// 	Attribute<Real>::Ptr coeffReal(Index row, Index col) {
	// 		Attribute<Real>::Getter get = [this, row, col]() -> Real {
	// 			return this->getByValue()(row,col).real();
	// 		};
	// 		return Attribute<Real>::make(get, mFlags, shared_from_this());
	// 		//Complex *ptr = &mData->data()[mData->cols() * row + col]; // Column major
	// 		//Real *realPart = &reinterpret_cast<Real*>(ptr)[0];
	// 		//return Attribute<Real>::make(&realPart, mFlags, shared_from_this());
	// 	}

	// 	Attribute<Real>::Ptr coeffImag(Index row, Index col) {
	// 		Attribute<Real>::Getter get = [this, row, col]() -> Real {
	// 			return this->getByValue()(row,col).imag();;
	// 		};
	// 		return Attribute<Real>::make(get, mFlags, shared_from_this());
	// 	}

	// 	Attribute<Real>::Ptr coeffMag(Index row, Index col) {
	// 		Attribute<Real>::Getter get = [this, row, col]() -> Real {
	// 			return Math::abs(this->get()(row,col));
	// 		};
	// 		return Attribute<Real>::make(get, mFlags, shared_from_this());
	// 		//Complex *ptr = &mData->data()[mData->cols() * row + col]; // Column major
	// 		//Real *realPart = &reinterpret_cast<Real*>(ptr)[0];
	// 		//return Attribute<Real>::make(&realPart, mFlags, shared_from_this());
	// 	}

	// 	Attribute<Real>::Ptr coeffPhase(Index row, Index col) {
	// 		Attribute<Real>::Getter get = [this, row, col]() -> Real {
	// 			return Math::phase(this->get()(row,col));
	// 		};
	// 		return Attribute<Real>::make(get, mFlags, shared_from_this());
	// 	}

	// };

	template<>
	String Attribute<Complex>::toString();

	template<>
	String Attribute<String>::toString();

	template<>
	String Attribute<MatrixComp>::toString();

	template<>
	String Attribute<Matrix>::toString();
}
