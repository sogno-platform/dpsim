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

	template<class U>
	concept Arithmetic = std::is_arithmetic<U>::value;

	enum UpdateTaskKind {
		UPDATE_ONCE,
		UPDATE_ON_GET,
		UPDATE_ON_SET,
		UPDATE_ON_SIMULATION_STEP,
	};

	template<class T>
	class Attribute;

	template<class T>
	class AttributeStatic;

	template<class T>
	class AttributeDynamic;

	template<class DependentType>
	class AttributeUpdateTaskBase {

	public:
		//FIXME: Why must this not be a pure virtual function, i. e. why can't this class be abstract?
		virtual void executeUpdate(std::shared_ptr<DependentType> &dependent) {
			throw TypeException();
		}; 
	};

	template<class DependentType, class... DependencyTypes>
	class AttributeUpdateTask :
		public AttributeUpdateTaskBase<DependentType>,
		public SharedFactory<AttributeUpdateTask<DependentType, DependencyTypes...>> {
	
	public:
		using Actor = std::function<void(std::shared_ptr<DependentType>&, std::shared_ptr<Attribute<DependencyTypes>>...)>;

	protected:
		std::tuple<std::shared_ptr<Attribute<DependencyTypes>>...> mDependencies;
		Actor mActorFunction;
		UpdateTaskKind mKind;

	public:
		AttributeUpdateTask(UpdateTaskKind kind, Actor &actorFunction, std::shared_ptr<Attribute<DependencyTypes>>... dependencies)
			: mKind(kind), mActorFunction(actorFunction), mDependencies(std::forward<std::shared_ptr<Attribute<DependencyTypes>>>(dependencies)...) {}

		virtual void executeUpdate(std::shared_ptr<DependentType> &dependent) override {
			mActorFunction(dependent, std::get<std::shared_ptr<Attribute<DependencyTypes>>...>(mDependencies));
		}
	};


	class AttributeBase {

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

		virtual String toString() = 0;

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
		public std::enable_shared_from_this<Attribute<T>> {

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

		virtual void set(T value) = 0;

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

		virtual String toString() override {
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

		template <class U>
		typename Attribute<U>::Ptr derive(
			int flags,
			typename AttributeUpdateTask<U, T>::Actor getter = AttributeUpdateTask<U, T>::Actor(),
			typename AttributeUpdateTask<U, T>::Actor setter = AttributeUpdateTask<U, T>::Actor()
		)
		{
			auto derivedAttribute = std::make_shared<AttributeDynamic<U>>(flags);
			if (setter) {
				derivedAttribute->addTask(UpdateTaskKind::UPDATE_ON_SET, AttributeUpdateTask<U, T>(UpdateTaskKind::UPDATE_ON_SET, setter, this->shared_from_this()));
			}
			if (getter) {
				derivedAttribute->addTask(UpdateTaskKind::UPDATE_ON_GET, AttributeUpdateTask<U, T>(UpdateTaskKind::UPDATE_ON_GET, getter, this->shared_from_this()));
			}
			return derivedAttribute;
		}

		template <class U>
		typename Attribute<U>::Ptr derive(
			typename AttributeUpdateTask<U, T>::Actor getter = AttributeUpdateTask<U, T>::Actor(),
			typename AttributeUpdateTask<U, T>::Actor setter = AttributeUpdateTask<U, T>::Actor()
		)
		{
			return derive<U>(this->mFlags, getter, setter);
		}

		std::shared_ptr<Attribute<Real>> deriveReal()
			requires std::same_as<T, CPS::Complex>
		{
			AttributeUpdateTask<CPS::Real, CPS::Complex>::Actor getter = [](std::shared_ptr<Real> dependent, std::shared_ptr<Attribute<Complex>> dependency) {
				*dependent = (**dependency).real();
			};
			AttributeUpdateTask<CPS::Real, CPS::Complex>::Actor setter = [](std::shared_ptr<Real> dependent, std::shared_ptr<Attribute<Complex>> dependency) {
				CPS::Complex currentValue = dependency->get();
				currentValue.real(*dependent);
				dependency->set(currentValue);
			};
			return derive<CPS::Real>(getter, setter);
		}

		std::shared_ptr<Attribute<Real>> deriveImag()
			requires std::same_as<T, CPS::Complex>
		{
			AttributeUpdateTask<CPS::Real, CPS::Complex>::Actor getter = [](std::shared_ptr<Real> dependent, Attribute<Complex>::Ptr dependency) {
				*dependent = (**dependency).imag();
			};
			AttributeUpdateTask<CPS::Real, CPS::Complex>::Actor setter = [](std::shared_ptr<Real> dependent, Attribute<Complex>::Ptr dependency) {
				CPS::Complex currentValue = dependency->get();
				currentValue.imag(*dependent);
				dependency->set(currentValue);
			};
			return derive<CPS::Real>(getter, setter);
		}

		std::shared_ptr<Attribute<Real>> deriveMag()
			requires std::same_as<T, CPS::Complex>
		{
			AttributeUpdateTask<CPS::Real, CPS::Complex>::Actor getter = [](std::shared_ptr<Real> dependent, Attribute<Complex>::Ptr dependency) {
				*dependent = Math::abs(**dependency);
			};
			AttributeUpdateTask<CPS::Real, CPS::Complex>::Actor setter = [](std::shared_ptr<Real> dependent, Attribute<Complex>::Ptr dependency) {
				CPS::Complex currentValue = dependency->get();
				dependency->set(Math::polar(*dependent, Math::phase(currentValue)));
			};
			return derive<CPS::Real>(getter, setter);
		}

		std::shared_ptr<Attribute<Real>> derivePhase()
			requires std::same_as<T, CPS::Complex>
		{
			AttributeUpdateTask<CPS::Real, CPS::Complex>::Actor getter = [](std::shared_ptr<Real> dependent, Attribute<Complex>::Ptr dependency) {
				*dependent = Math::phase(**dependency);
			};
			AttributeUpdateTask<CPS::Real, CPS::Complex>::Actor setter = [](std::shared_ptr<Real> dependent, Attribute<Complex>::Ptr dependency) {
				CPS::Complex currentValue = dependency->get();
				dependency->set(Math::polar(Math::abs(currentValue), *dependent));
			};
			return derive<CPS::Real>(getter, setter);
		}

		std::shared_ptr<Attribute<T>> deriveScaled(T scale)
			requires std::same_as<T, CPS::Complex> || std::same_as<T, CPS::Real>
		{
			typename AttributeUpdateTask<T, T>::Actor getter = [scale](std::shared_ptr<T> dependent, Attribute<T>::Ptr dependency) {
				*dependent = scale * (**dependency);
			};
			typename AttributeUpdateTask<T, T>::Actor setter = [scale](std::shared_ptr<T> dependent, Attribute<T>::Ptr dependency) {
				dependency->set((*dependent) / scale);
			};
			return derive<T>(getter, setter);
		}

		template<class U>
		std::shared_ptr<Attribute<U>> deriveCoeff(CPS::MatrixVar<U>::Index row, CPS::MatrixVar<U>::Index column)
			requires std::same_as<T, CPS::MatrixVar<U>>
		{
			typename AttributeUpdateTask<U, T>::Actor getter = [row, column](std::shared_ptr<U> dependent, Attribute<T>::Ptr dependency) {
				*dependent = (**dependency)(row, column);
			};
			typename AttributeUpdateTask<U, T>::Actor setter = [row, column](std::shared_ptr<U> dependent, Attribute<T>::Ptr dependency) {
				CPS::MatrixVar<U> currentValue = dependency->get();
				currentValue(row, column) = *dependent;
				dependency->set(currentValue);
			};
			return derive<U>(getter, setter);
		}

	};

	template<class T>
	class AttributeStatic :
		public Attribute<T>,
		public SharedFactory<AttributeStatic<T>> { 
		friend class SharedFactory<AttributeStatic<T>>;

	public:
		AttributeStatic(int flags = Flags::read) :
			Attribute<T>(flags) { }

		virtual void set(T value) override {
			if (this->mFlags & Flags::write) {
				*this->mData = value;
			} else {
				throw AccessException(); 
			}
		};

		virtual const T& get() override {
			if (this->mFlags & Flags::read) {
				return *this->mData;
			}
			else
				throw AccessException();
		};
	};

	template<class T>
	class AttributeDynamic :
		public Attribute<T>,
		public SharedFactory<AttributeDynamic<T>> { 
		friend class SharedFactory<AttributeDynamic<T>>;

	
	protected:
		std::vector<AttributeUpdateTaskBase<T>> updateTasksOnGet;
		std::vector<AttributeUpdateTaskBase<T>> updateTasksOnSet;

	public:
		AttributeDynamic(int flags = Flags::read) :
			Attribute<T>(flags) { }

		void addTask(UpdateTaskKind kind, AttributeUpdateTaskBase<T> task) {
			switch (kind) {
				case UpdateTaskKind::UPDATE_ONCE:
					throw InvalidArgumentException();
				case UpdateTaskKind::UPDATE_ON_GET:
					updateTasksOnGet.push_back(task);
					break;
				case UpdateTaskKind::UPDATE_ON_SET:
					updateTasksOnSet.push_back(task);
					break;
				case UpdateTaskKind::UPDATE_ON_SIMULATION_STEP:
					throw InvalidArgumentException();
			};
		}

		virtual void set(T value) override {
			if (this->mFlags & Flags::write) {
				*this->mData = value;
				for(auto task : updateTasksOnSet) {
					task.executeUpdate(this->mData);
				}
			} else {
				throw AccessException(); 
			}
		};

		virtual const T& get() override {
			if (this->mFlags & Flags::read) {
				for(auto task : updateTasksOnGet) {
					task.executeUpdate(this->mData);
				}
				return *this->mData;
			}
			else
				throw AccessException();
		};
	};

	template<>
	String Attribute<Complex>::toString();

	template<>
	String Attribute<String>::toString();

	template<>
	String Attribute<MatrixComp>::toString();

	template<>
	String Attribute<Matrix>::toString();
}
