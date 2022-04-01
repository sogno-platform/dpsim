/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once
#include <iostream>
#include <set>

#include <cps/Definitions.h>
#include <cps/PtrFactory.h>
#include <cps/MathUtils.h>
#include <cps/Config.h>
namespace CPS {

	enum UpdateTaskKind {
		UPDATE_ONCE,
		UPDATE_ON_GET,
		UPDATE_ON_SET,
		UPDATE_ON_SIMULATION_STEP,
	};

	template<class T>
	class AttributePointer {
		public:
			AttributePointer() : mPtr() {};
			AttributePointer(const AttributePointer& r) = default;
			AttributePointer(std::shared_ptr<T> ptr) : mPtr(ptr) {};

			template<class U>
			AttributePointer(AttributePointer<U> ptr) : mPtr() {
				this->mPtr = ptr.getPtr();
			};

			template<class U>
			AttributePointer(std::shared_ptr<U> ptr) : mPtr(ptr) {};

			AttributePointer& operator=(const AttributePointer& r) noexcept {
				this->mPtr = r.getPtr();
				return *this;
			};

			template< class U >
			AttributePointer& operator=(const AttributePointer<U>& r) noexcept {
				this->mPtr = r.getPtr();
				return *this;
			};

			AttributePointer& operator=(AttributePointer&& r) {
				this->mPtr = r.getPtr();
				return *this;
			} 

			template<class U>
			AttributePointer& operator=(AttributePointer<U>&& r) {
				this->mPtr = r.getPtr();
				return *this;
			} 

			template<class U>
			bool operator==(AttributePointer<U>&& rhs) {
				return this->mPtr == rhs.getPtr();
			}

			T& operator*() const noexcept {
				return *mPtr;
			}

			T* operator->() const noexcept {
				return mPtr.operator->();
			}

			std::shared_ptr<T> getPtr() const {
				return mPtr;
			}

			template<class U>
			bool operator<(const AttributePointer<U>& rhs) const noexcept {
				return this->mPtr < rhs.getPtr();
			}

			template<class U>
			bool operator>(const AttributePointer<U>& rhs) const noexcept {
				return this->mPtr > rhs.getPtr();
			}

			template<class U>
			bool operator==(const AttributePointer<U>& rhs) const noexcept {
				return this->mPtr == rhs.getPtr();
			}

			template<class U>
			bool operator!=(const AttributePointer<U>& rhs) const noexcept {
				return this->mPtr != rhs.getPtr();
			}

		private:
			std::shared_ptr<T> mPtr;
	};

	class AttributeBase {
	public:
		typedef AttributePointer<AttributeBase> Ptr;
		typedef std::vector<Ptr> List;
		typedef std::set<Ptr> Set;
		typedef std::map<String, Ptr> Map;

		virtual String toString() = 0;
		virtual bool isStatic() const = 0;
		virtual ~AttributeBase() = default;
		virtual void appendDependencies(AttributeBase::Set *deps) = 0;

		virtual AttributeBase::Set getDependencies() final {
			AttributeBase::Set deps = AttributeBase::Set();
			this->appendDependencies(&deps);
			return deps;
		}
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
		typedef std::shared_ptr<AttributeUpdateTaskBase<DependentType>> Ptr;

		virtual void executeUpdate(std::shared_ptr<DependentType> &dependent) = 0;
		virtual AttributeBase::List getDependencies() = 0;
		virtual ~AttributeUpdateTaskBase() = default;
	};

	template<class DependentType, class... DependencyTypes>
	class AttributeUpdateTask :
		public AttributeUpdateTaskBase<DependentType>,
		public SharedFactory<AttributeUpdateTask<DependentType, DependencyTypes...>> {
	
	public:
		using Actor = std::function<void(std::shared_ptr<DependentType>&, typename Attribute<DependencyTypes>::Ptr...)>;

	protected:
		std::tuple<typename Attribute<DependencyTypes>::Ptr...> mDependencies;
		Actor mActorFunction;
		UpdateTaskKind mKind;

	public:
		AttributeUpdateTask(UpdateTaskKind kind, Actor &actorFunction, typename Attribute<DependencyTypes>::Ptr... dependencies)
			: mDependencies(std::forward<typename Attribute<DependencyTypes>::Ptr>(dependencies)...), mActorFunction(actorFunction), mKind(kind) {}

		virtual void executeUpdate(std::shared_ptr<DependentType> &dependent) override {
			mActorFunction(dependent, std::get<typename Attribute<DependencyTypes>::Ptr...>(mDependencies));
		}

		virtual AttributeBase::List getDependencies() override {
			return std::apply([](auto&&... elems){
				return std::vector<AttributeBase::Ptr>{std::forward<decltype(elems)>(elems)...};
			}, mDependencies);
		};
	};

	template<class T>
	class Attribute :
		public AttributeBase,
		public std::enable_shared_from_this<Attribute<T>> {

	protected:
		std::shared_ptr<T> mData;

	public:
		typedef T Type;
		typedef AttributePointer<Attribute<T>> Ptr;

		Attribute(T initialValue = T()) :
			AttributeBase(), mData(std::make_shared<T>()) {
				*mData = initialValue;
			}

		static Attribute<T>::Ptr create(String name, AttributeBase::Map &attrMap, T intitialValue = T()) {
			Attribute<T>::Ptr newAttr = AttributePointer<Attribute<T>>(AttributeStatic<T>::make(intitialValue));
			attrMap[name] = newAttr;
			return newAttr;
		}

		static Attribute<T>::Ptr createDynamic(String name, AttributeBase::Map &attrMap) {
			Attribute<T>::Ptr newAttr = AttributePointer<Attribute<T>>(AttributeDynamic<T>::make());
			attrMap[name] = newAttr;
			return newAttr;
		}

		virtual void set(T value) = 0;

		virtual T& get() = 0;

		virtual void setReference(Attribute<T>::Ptr reference) = 0;

		virtual const std::shared_ptr<T> asRawPointer() {
			return this->mData;
		}

		/// Fallback method for all attribute types not covered by the specifications in Attribute.cpp
		virtual String toString() override {
			std::stringstream ss;
			ss << this->get();
			return ss.str();
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
		T& operator*(){
			return this->get();
		}

		// /// Do not use!
		// /// Only used for Eigen Matrix - Sundials N_Vector interfacing in N_VSetArrayPointer
		// operator T&() {
		// 	return this->get()->get();
		// }

		template <class U>
		typename Attribute<U>::Ptr derive(
			typename AttributeUpdateTask<U, T>::Actor getter = typename AttributeUpdateTask<U, T>::Actor(),
			typename AttributeUpdateTask<U, T>::Actor setter = typename AttributeUpdateTask<U, T>::Actor()
		)
		{
			typename Attribute<U>::Ptr derivedAttribute = std::make_shared<AttributeDynamic<U>>();
			if (setter) {
				derivedAttribute->addTask(UpdateTaskKind::UPDATE_ON_SET, AttributeUpdateTask<U, T>::make(UpdateTaskKind::UPDATE_ON_SET, setter, Attribute<T>::Ptr(this->shared_from_this())));
			}
			if (getter) {
				derivedAttribute->addTask(UpdateTaskKind::UPDATE_ON_GET, AttributeUpdateTask<U, T>::make(UpdateTaskKind::UPDATE_ON_GET, getter, Attribute<T>::Ptr(this->shared_from_this())));
			}
			return derivedAttribute;
		}

		template <typename U = T, std::enable_if_t<std::is_same_v<Complex, U>, bool> = true>
		std::shared_ptr<Attribute<Real>> deriveReal()
			// requires std::same_as<T, CPS::Complex> //CPP20
		{
			AttributeUpdateTask<CPS::Real, CPS::Complex>::Actor getter = [](std::shared_ptr<Real> &dependent, std::shared_ptr<Attribute<Complex>> dependency) {
				*dependent = (**dependency).real();
			};
			AttributeUpdateTask<CPS::Real, CPS::Complex>::Actor setter = [](std::shared_ptr<Real> &dependent, std::shared_ptr<Attribute<Complex>> dependency) {
				CPS::Complex currentValue = dependency->get();
				currentValue.real(*dependent);
				dependency->set(currentValue);
			};
			return derive<CPS::Real>(getter, setter);
		}

		template <typename U = T, std::enable_if_t<std::is_same_v<Complex, U>, bool> = true>
		std::shared_ptr<Attribute<Real>> deriveImag()
			// requires std::same_as<T, CPS::Complex> //CPP20
		{
			AttributeUpdateTask<CPS::Real, CPS::Complex>::Actor getter = [](std::shared_ptr<Real> &dependent, Attribute<Complex>::Ptr dependency) {
				*dependent = (**dependency).imag();
			};
			AttributeUpdateTask<CPS::Real, CPS::Complex>::Actor setter = [](std::shared_ptr<Real> &dependent, Attribute<Complex>::Ptr dependency) {
				CPS::Complex currentValue = dependency->get();
				currentValue.imag(*dependent);
				dependency->set(currentValue);
			};
			return derive<CPS::Real>(getter, setter);
		}

		template <typename U = T, std::enable_if_t<std::is_same_v<Complex, U>, bool> = true>
		std::shared_ptr<Attribute<Real>> deriveMag()
			// requires std::same_as<T, CPS::Complex> //CPP20
		{
			AttributeUpdateTask<CPS::Real, CPS::Complex>::Actor getter = [](std::shared_ptr<Real> &dependent, Attribute<Complex>::Ptr dependency) {
				*dependent = Math::abs(**dependency);
			};
			AttributeUpdateTask<CPS::Real, CPS::Complex>::Actor setter = [](std::shared_ptr<Real> &dependent, Attribute<Complex>::Ptr dependency) {
				CPS::Complex currentValue = dependency->get();
				dependency->set(Math::polar(*dependent, Math::phase(currentValue)));
			};
			return derive<CPS::Real>(getter, setter);
		}

		template <typename U = T, std::enable_if_t<std::is_same_v<Complex, U>, bool> = true>
		std::shared_ptr<Attribute<Real>> derivePhase()
			// requires std::same_as<T, CPS::Complex> //CPP20
		{
			AttributeUpdateTask<CPS::Real, CPS::Complex>::Actor getter = [](std::shared_ptr<Real> &dependent, Attribute<Complex>::Ptr dependency) {
				*dependent = Math::phase(**dependency);
			};
			AttributeUpdateTask<CPS::Real, CPS::Complex>::Actor setter = [](std::shared_ptr<Real> &dependent, Attribute<Complex>::Ptr dependency) {
				CPS::Complex currentValue = dependency->get();
				dependency->set(Math::polar(Math::abs(currentValue), *dependent));
			};
			return derive<CPS::Real>(getter, setter);
		}

		template <typename U = T, std::enable_if_t<std::is_same_v<Real, U> || std::is_same_v<Complex, U>, bool> = true>
		std::shared_ptr<Attribute<T>> deriveScaled(T scale)
			// requires std::same_as<T, CPS::Complex> || std::same_as<T, CPS::Real> //CPP20
		{
			typename AttributeUpdateTask<T, T>::Actor getter = [scale](std::shared_ptr<T> &dependent, Attribute<T>::Ptr dependency) {
				*dependent = scale * (**dependency);
			};
			typename AttributeUpdateTask<T, T>::Actor setter = [scale](std::shared_ptr<T> &dependent, Attribute<T>::Ptr dependency) {
				dependency->set((*dependent) / scale);
			};
			return derive<T>(getter, setter);
		}

		template <class U, class V = T, std::enable_if_t<std::is_same_v<CPS::MatrixVar<U>, V>, bool> = true>
		std::shared_ptr<Attribute<U>> deriveCoeff(typename CPS::MatrixVar<U>::Index row, typename CPS::MatrixVar<U>::Index column)
			// requires std::same_as<T, CPS::MatrixVar<U>> //CPP20
		{
			typename AttributeUpdateTask<U, T>::Actor getter = [row, column](std::shared_ptr<U> &dependent, Attribute<T>::Ptr dependency) {
				*dependent = (**dependency)(row, column);
			};
			typename AttributeUpdateTask<U, T>::Actor setter = [row, column](std::shared_ptr<U> &dependent, Attribute<T>::Ptr dependency) {
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
		AttributeStatic(T initialValue = T()) :
			Attribute<T>(initialValue) { }

		virtual void set(T value) override {
			*this->mData = value;
		};

		virtual T& get() override {
			return *this->mData;
		};

		virtual bool isStatic() const override {
			return true;
		}

		virtual void setReference(typename Attribute<T>::Ptr reference) override {
			throw TypeException();
		}

		virtual void appendDependencies(AttributeBase::Set *deps) override {
			deps->insert(this->shared_from_this());
		}
	};

	template<class T>
	class AttributeDynamic :
		public Attribute<T>,
		public SharedFactory<AttributeDynamic<T>> { 
		friend class SharedFactory<AttributeDynamic<T>>;

	protected:
		std::vector<typename AttributeUpdateTaskBase<T>::Ptr> updateTasksOnce;
		std::vector<typename AttributeUpdateTaskBase<T>::Ptr> updateTasksOnGet;
		std::vector<typename AttributeUpdateTaskBase<T>::Ptr> updateTasksOnSet;

	public:
		AttributeDynamic(T initialValue = T()) :
			Attribute<T>(initialValue) { }

		void addTask(UpdateTaskKind kind, typename AttributeUpdateTaskBase<T>::Ptr task) {
			switch (kind) {
				case UpdateTaskKind::UPDATE_ONCE:
					updateTasksOnce.push_back(task);
					///THISISBAD: This is probably not the right time to run this kind of task
					task->executeUpdate(this->mData);
					break;
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

		void clearTasks(UpdateTaskKind kind) {
			switch (kind) {
				case UpdateTaskKind::UPDATE_ONCE:
					updateTasksOnce.clear();
					break;
				case UpdateTaskKind::UPDATE_ON_GET:
					updateTasksOnGet.clear();
					break;
				case UpdateTaskKind::UPDATE_ON_SET:
					updateTasksOnSet.clear();
					break;
				case UpdateTaskKind::UPDATE_ON_SIMULATION_STEP:
					throw InvalidArgumentException();
			};
		}

		void clearAllTasks() {
			updateTasksOnce.clear();
			updateTasksOnGet.clear();
			updateTasksOnSet.clear();
		}

		virtual void setReference(typename Attribute<T>::Ptr reference) override {
			typename AttributeUpdateTask<T, T>::Actor getter = [](std::shared_ptr<T> &dependent, typename Attribute<T>::Ptr dependency) {
				dependent = dependency->asRawPointer();
			};
			this->clearAllTasks();
			if(reference->isStatic()) {
				this->addTask(UpdateTaskKind::UPDATE_ONCE, AttributeUpdateTask<T, T>::make(UpdateTaskKind::UPDATE_ONCE, getter, reference));
			} else {
				this->addTask(UpdateTaskKind::UPDATE_ON_GET, AttributeUpdateTask<T, T>::make(UpdateTaskKind::UPDATE_ON_GET, getter, reference));
			}
		}

		virtual void set(T value) override {
			*this->mData = value;
			for(typename AttributeUpdateTaskBase<T>::Ptr task : updateTasksOnSet) {
				task->executeUpdate(this->mData);
			}
		};

		virtual T& get() override {
			for(typename AttributeUpdateTaskBase<T>::Ptr task : updateTasksOnGet) {
				task->executeUpdate(this->mData);
			}
			return *this->mData;
		};

		virtual bool isStatic() const override {
			return false;
		}

		/// This will recursively collect all attributes this attribute depends on, either in the UPDATE_ONCE or the UPDATE_ON_GET tasks.
		/// This is done by performing a Depth-First-Search on the dependency graph where the task dependencies of each attribute are the outgoing edges.
		/// The `deps` set contains all the nodes that have already been visited in the graph
		virtual void appendDependencies(AttributeBase::Set *deps) override {
			deps->insert(this->shared_from_this());
			
			AttributeBase::Set newDeps = AttributeBase::Set();
			for (typename AttributeUpdateTaskBase<T>::Ptr task : updateTasksOnce) {
				AttributeBase::List taskDeps = task->getDependencies();
				newDeps.insert(taskDeps.begin(), taskDeps.end());
			}

			for (typename AttributeUpdateTaskBase<T>::Ptr task : updateTasksOnGet) {
				AttributeBase::List taskDeps = task->getDependencies();
				newDeps.insert(taskDeps.begin(), taskDeps.end());
			}

			for (auto dependency : newDeps) {
				dependency->appendDependencies(deps);
			}
		}
	};

	template<>
	String Attribute<Real>::toString();

	template<>
	String Attribute<Complex>::toString();

	template<>
	String Attribute<String>::toString();

}


