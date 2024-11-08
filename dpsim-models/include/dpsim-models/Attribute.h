/* Copyright 2017-2022 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once
#include <iostream>
#include <set>

#include <dpsim-models/Config.h>
#include <dpsim-models/Definitions.h>
#include <dpsim-models/MathUtils.h>
#include <dpsim-models/PtrFactory.h>
namespace CPS {

/**
	 * Enum to describe when a given ´UpdateTask` should be executed.
	 * UPDATE_ONCE tasks are currently executed instantly after they have been set, and then never again.
	 * UPDATE_ON_GET tasks are executed whenever the attribute is dereferenced.
	 * UPDATE_ON_SET tasks are only executed whenever the `set` method is called. Since `get` provides mutable references,
	 * this is not guaranteed to happen on every change to the attribute!
	 * UPDATE_ON_SIMULATION_STEP is currently unused.
	 * */
enum UpdateTaskKind {
  UPDATE_ONCE,
  UPDATE_ON_GET,
  UPDATE_ON_SET,
  UPDATE_ON_SIMULATION_STEP,
};

template <class T> class Attribute;

template <class T> class AttributeStatic;

template <class T> class AttributeDynamic;

/**
	 * Custom pointer class for storing attributes as member variables and in the `mAttributes` attribute map.
	 * Using this type over the normal `std::shared_ptr` allows for disabling certain operator overloads, e.g. the comparison with the nullptr / the number 0
	 * that is possible with shared ptrs. Since attribute pointers rarely need to be compared with the nullptr, disabling the implicit comparison allows for
	 * detecting more errors at compile time. Explicit comparison still remains possible via the `getPtr` method.
	 * */
template <class T> class AttributePointer {
public:
  using element_type = T;

  AttributePointer() : mPtr(){};
  AttributePointer(const AttributePointer &r) = default;
  AttributePointer(std::shared_ptr<T> ptr) : mPtr(ptr){};
  AttributePointer(std::nullptr_t ptr) : mPtr(ptr){};
  explicit AttributePointer(T *ptr) : mPtr(ptr){};

  template <class U> AttributePointer(AttributePointer<U> ptr) : mPtr() {
    this->mPtr = ptr.getPtr();
  };

  template <class U> AttributePointer(std::shared_ptr<U> ptr) : mPtr(ptr){};

  AttributePointer &operator=(const AttributePointer &r) noexcept {
    this->mPtr = r.getPtr();
    return *this;
  };

  template <class U>
  AttributePointer &operator=(const AttributePointer<U> &r) noexcept {
    this->mPtr = r.getPtr();
    return *this;
  };

  AttributePointer &operator=(AttributePointer &&r) {
    this->mPtr = r.getPtr();
    return *this;
  }

  template <class U> AttributePointer &operator=(AttributePointer<U> &&r) {
    this->mPtr = r.getPtr();
    return *this;
  }

  T &operator*() const noexcept { return *mPtr; }

  T *operator->() const noexcept { return mPtr.operator->(); }

  T *get() const { return mPtr.get(); }

  std::shared_ptr<T> getPtr() const { return mPtr; }

  bool isNull() const { return mPtr == nullptr; }

  /*
			These (implicit) comparison operators are disabled to avoid accidentally comparing pointers instead of attribute values.
			When a pointer comparison is necessary, this can be done via the `getPtr` method or by using the `AttributeCmp` and `AttributeEq` structs.

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
			}*/

private:
  std::shared_ptr<T> mPtr;
};

/**
	 * Struct providing an (explicit) comparison function for Attribute Pointers. Can be used in STL containers.
	 * */
template <class T> struct AttributeCmp {
  bool operator()(CPS::AttributePointer<T> a,
                  CPS::AttributePointer<T> b) const {
    return a.getPtr() < b.getPtr();
  }
};

/**
	 * Struct providing an (explicit) equals function for Attribute Pointers. Can be used in STL containers.
	 * */
template <class T> struct AttributeEq {
  bool operator()(CPS::AttributePointer<T> a,
                  CPS::AttributePointer<T> b) const {
    return a.getPtr() == b.getPtr();
  }
};

/**
	 * Base class for all Attribute types. Can be used in STL containers to hide the template information.
	 * */
class AttributeBase {
public:
  typedef AttributePointer<AttributeBase> Ptr;
  typedef std::vector<Ptr> List;
  typedef std::set<Ptr, AttributeCmp<AttributeBase>> Set;
  typedef std::map<String, Ptr> Map;

  /**
		 * Display this attribute's value as a string
		 * */
  virtual String toString() = 0;

  /**
		 * Check whether this is a static or dynamic attribute
		 * @return true for instances of `AttributeStatic`, false for instances of `AttributeDynamic`
		 * */
  virtual bool isStatic() const = 0;

  virtual ~AttributeBase() = default;

  /**
		 * @brief Copy the attribute value of `copyFrom` onto this attribute
		 * @return true if the copy operation was successful, false otherwise
		 */
  virtual bool copyValue(AttributeBase::Ptr copyFrom) = 0;

  /**
		 * @brief Get the type of this attribute
		 * @return std::type_info
		 */
  virtual const std::type_info &getType() = 0;

  /**
		 * @brief Generates a new attribute of the same type and copies the current value in the heap. Does not copy any dependency relations!
		 * @return Pointer to the copied attribute
		 */
  virtual AttributeBase::Ptr cloneValueOntoNewAttribute() = 0;

  /**
		 * Append all dependencies of this attribute to the given set.
		 * For static attributes, this will only append `this`, for dynamic attributes, it will recursively collect and append
		 * all dependencies.
		 * */
  virtual void appendDependencies(AttributeBase::Set *deps) = 0;

  /**
		 * Get a set of all attributes this attribute depends on. For static attributes, this set will only contain `this`.
		 * For dynamic attributes, this will recursively collect all dependency attributes.
		 * */
  virtual AttributeBase::Set getDependencies() final {
    AttributeBase::Set deps = AttributeBase::Set();
    this->appendDependencies(&deps);
    return deps;
  }
};

/**
	 * Base class for all AttributeUpdateTasks. Enables storing tasks in an STL list independent of the dependency types.
	 * */
template <class DependentType> class AttributeUpdateTaskBase {

public:
  typedef std::shared_ptr<AttributeUpdateTaskBase<DependentType>> Ptr;

  virtual void executeUpdate(std::shared_ptr<DependentType> &dependent) = 0;
  virtual AttributeBase::List getDependencies() = 0;
  virtual ~AttributeUpdateTaskBase() = default;
};

/**
	 * Specialized class for AttributeUpdateTasks that includes information about the types of attributes this task depends on.
	 * @param DependentType The type of the attribute which is updated by this task
	 * @param DependencyTypes List of the types of the attributes whose values are used to update the dependent attribute
	 * */
template <class DependentType, class... DependencyTypes>
class AttributeUpdateTask
    : public AttributeUpdateTaskBase<DependentType>,
      public SharedFactory<
          AttributeUpdateTask<DependentType, DependencyTypes...>> {

public:
  using Actor =
      std::function<void(std::shared_ptr<DependentType> &,
                         typename Attribute<DependencyTypes>::Ptr...)>;

protected:
  std::tuple<typename Attribute<DependencyTypes>::Ptr...> mDependencies;
  Actor mActorFunction;
  UpdateTaskKind mKind;

public:
  AttributeUpdateTask(UpdateTaskKind kind, Actor &actorFunction,
                      typename Attribute<DependencyTypes>::Ptr... dependencies)
      : mDependencies(std::forward<typename Attribute<DependencyTypes>::Ptr>(
            dependencies)...),
        mActorFunction(actorFunction), mKind(kind) {}

  virtual void
  executeUpdate(std::shared_ptr<DependentType> &dependent) override {
    mActorFunction(
        dependent,
        std::get<typename Attribute<DependencyTypes>::Ptr...>(mDependencies));
  }

  /**
		 * Returns all dependency elements in the `mDependency` tuple in a list over AttributeBase pointers.
		 * */
  virtual AttributeBase::List getDependencies() override {
    return std::apply(
        [](auto &&...elems) {
          return std::vector<AttributeBase::Ptr>{
              std::forward<decltype(elems)>(elems)...};
        },
        mDependencies);
  };
};

/**
	 * Main Attribute class. The template class `T` holds the attribute's type. This is used as the type for all attribute member variables.
	 * @param T The type of this attribute
	 * */
template <class T>
class Attribute : public AttributeBase,
                  public std::enable_shared_from_this<Attribute<T>> {

protected:
  std::shared_ptr<T> mData;

public:
  using Type = T;
  using Ptr = AttributePointer<Attribute<T>>;

  Attribute(T initialValue = T())
      : AttributeBase(), mData(std::make_shared<T>()) {
    *mData = initialValue;
  }

  /**
		 * Manually set the attribute to the given value. For dynamic attributes, this will trigger the UPDATE_ON_SET tasks for updating any
		 * dependency attributes.
		 * */
  virtual void set(T value) = 0;

  /**
		 * Get a mutable reference to the attribute's underlying data. This method is also called when dereferencing an attribute using the * operator
		 * */
  virtual T &get() = 0;

  /**
		 * Convenience method for setting this attribute to always equal another attribute.
		 * When `this` is dynamic, this will set up an UPDATE_ONCE task that sets this attribute's data pointer to equal the data pointer of the referenced attribute.
		 * If `this` is static, calling this method will result in a runtime error.
		 * @param reference The attribute which's value will be adapted
		 * */
  virtual void setReference(Attribute<T>::Ptr reference) = 0;

  /**
		 * Exposing the underlying shared_ptr for this attribute's data. Used to create reference relations between two attributes.
		 * @return The shared_ptr to this attribute's underlying data
		 * */
  virtual std::shared_ptr<T> asRawPointer() = 0;

  /// Fallback method for all attribute types not covered by the specifications in Attribute.cpp
  String toString() override {
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
  operator const T &() { return this->get(); }

  /// @brief User-defined dereference operator
  ///
  /// Allows easier access to the attribute's underlying data
  T &operator*() { return this->get(); }

  /**
		 * @brief Copy the attribute value of `copyFrom` onto this attribute
		 * @return true if the copy operation was successful, false otherwise
		 */
  bool copyValue(AttributeBase::Ptr copyFrom) override {
    Attribute<T>::Ptr copyFromTyped =
        std::dynamic_pointer_cast<Attribute<T>>(copyFrom.getPtr());
    if (copyFromTyped.getPtr() == nullptr) {
      return false;
    }
    this->set(**copyFromTyped);
    return true;
  }

  /**
		 * @brief Get the type of this attribute
		 * @return std::type_info
		 */
  const std::type_info &getType() override { return typeid(T); }

  /**
		 * @brief Generates a new attribute of the same type and copies the current value in the heap. Does not copy any dependency relations!
		 * @return Pointer to the copied attribute
		 */
  AttributeBase::Ptr cloneValueOntoNewAttribute() override {
    return AttributePointer<AttributeBase>(
        AttributeStatic<T>::make(this->get()));
    // TODO: This is in the real time path. We should not use heap here.
  };

  /**
		 * General method for deriving a new attribute from this attribute. Custom getter and setter functions have to be provided. The newly created
		 * attribute will only depend on this attribute in a 1:1 relationship.
		 * @param U The type of the newly derived attribute
		 * @param getter The getter actor function to use for updating the derived attribute
		 * @param setter The setter actor function for updating `this` when the derived attribute is changed
		 * @return a newly created attribute of type `U` which will calculate its value using the provided getter and update `this` on changes using the provided setter
		 * */
  template <class U>
  typename Attribute<U>::Ptr
  derive(typename AttributeUpdateTask<U, T>::Actor getter =
             typename AttributeUpdateTask<U, T>::Actor(),
         typename AttributeUpdateTask<U, T>::Actor setter =
             typename AttributeUpdateTask<U, T>::Actor()) {
    auto derivedAttribute = std::make_shared<AttributeDynamic<U>>();
    if (setter) {
      derivedAttribute->addTask(
          UpdateTaskKind::UPDATE_ON_SET,
          AttributeUpdateTask<U, T>::make(
              UpdateTaskKind::UPDATE_ON_SET, setter,
              Attribute<T>::Ptr(this->shared_from_this())));
    }
    if (getter) {
      derivedAttribute->addTask(
          UpdateTaskKind::UPDATE_ON_GET,
          AttributeUpdateTask<U, T>::make(
              UpdateTaskKind::UPDATE_ON_GET, getter,
              Attribute<T>::Ptr(this->shared_from_this())));
    }
    return derivedAttribute;
  }

  /**
		 * Convenience method for deriving the real part of a complex attribute
		 * @return a new attribute whose value will always equal the real part of `this`
		 * */
  template <typename U = T,
            std::enable_if_t<std::is_same_v<Complex, U>, bool> = true>
  AttributePointer<Attribute<Real>> deriveReal()
  // requires std::same_as<T, CPS::Complex> //CPP20
  {
    AttributeUpdateTask<CPS::Real, CPS::Complex>::Actor getter =
        [](std::shared_ptr<Real> &dependent,
           typename Attribute<Complex>::Ptr dependency) {
          *dependent = (**dependency).real();
        };
    AttributeUpdateTask<CPS::Real, CPS::Complex>::Actor setter =
        [](std::shared_ptr<Real> &dependent,
           typename Attribute<Complex>::Ptr dependency) {
          CPS::Complex currentValue = dependency->get();
          currentValue.real(*dependent);
          dependency->set(currentValue);
        };
    return derive<CPS::Real>(getter, setter);
  }

  /**
		 * Convenience method for deriving the imaginary part of a complex attribute
		 * @return a new attribute whose value will always equal the imaginary part of `this`
		 * */
  template <typename U = T,
            std::enable_if_t<std::is_same_v<Complex, U>, bool> = true>
  AttributePointer<Attribute<Real>> deriveImag()
  // requires std::same_as<T, CPS::Complex> //CPP20
  {
    AttributeUpdateTask<CPS::Real, CPS::Complex>::Actor getter =
        [](std::shared_ptr<Real> &dependent,
           Attribute<Complex>::Ptr dependency) {
          *dependent = (**dependency).imag();
        };
    AttributeUpdateTask<CPS::Real, CPS::Complex>::Actor setter =
        [](std::shared_ptr<Real> &dependent,
           Attribute<Complex>::Ptr dependency) {
          CPS::Complex currentValue = dependency->get();
          currentValue.imag(*dependent);
          dependency->set(currentValue);
        };
    return derive<CPS::Real>(getter, setter);
  }

  /**
		 * Convenience method for deriving the magnitude of a complex attribute
		 * @return a new attribute whose value will always equal the magnitude of `this`
		 * */
  template <typename U = T,
            std::enable_if_t<std::is_same_v<Complex, U>, bool> = true>
  AttributePointer<Attribute<Real>> deriveMag()
  // requires std::same_as<T, CPS::Complex> //CPP20
  {
    AttributeUpdateTask<CPS::Real, CPS::Complex>::Actor getter =
        [](std::shared_ptr<Real> &dependent,
           Attribute<Complex>::Ptr dependency) {
          *dependent = Math::abs(**dependency);
        };
    AttributeUpdateTask<CPS::Real, CPS::Complex>::Actor setter =
        [](std::shared_ptr<Real> &dependent,
           Attribute<Complex>::Ptr dependency) {
          CPS::Complex currentValue = dependency->get();
          dependency->set(Math::polar(*dependent, Math::phase(currentValue)));
        };
    return derive<CPS::Real>(getter, setter);
  }

  /**
		 * Convenience method for deriving the phase of a complex attribute
		 * @return a new attribute whose value will always equal the phase of `this`
		 * */
  template <typename U = T,
            std::enable_if_t<std::is_same_v<Complex, U>, bool> = true>
  AttributePointer<Attribute<Real>> derivePhase()
  // requires std::same_as<T, CPS::Complex> //CPP20
  {
    AttributeUpdateTask<CPS::Real, CPS::Complex>::Actor getter =
        [](std::shared_ptr<Real> &dependent,
           Attribute<Complex>::Ptr dependency) {
          *dependent = Math::phase(**dependency);
        };
    AttributeUpdateTask<CPS::Real, CPS::Complex>::Actor setter =
        [](std::shared_ptr<Real> &dependent,
           Attribute<Complex>::Ptr dependency) {
          CPS::Complex currentValue = dependency->get();
          dependency->set(Math::polar(Math::abs(currentValue), *dependent));
        };
    return derive<CPS::Real>(getter, setter);
  }

  /**
		 * Convenience method for deriving an attribute whose value is always scaled by `scale`
		 * @param scale The scaling to apply to the attribute's value
		 * @return a new attribute whose value will always equal the value of `this` but scaled by `scale`
		 * */
  template <typename U = T, std::enable_if_t<std::is_same_v<Real, U> ||
                                                 std::is_same_v<Complex, U>,
                                             bool> = true>
  AttributePointer<Attribute<T>> deriveScaled(T scale)
  // requires std::same_as<T, CPS::Complex> || std::same_as<T, CPS::Real> //CPP20
  {
    typename AttributeUpdateTask<T, T>::Actor getter =
        [scale](std::shared_ptr<T> &dependent, Attribute<T>::Ptr dependency) {
          *dependent = scale * (**dependency);
        };
    typename AttributeUpdateTask<T, T>::Actor setter =
        [scale](std::shared_ptr<T> &dependent, Attribute<T>::Ptr dependency) {
          dependency->set((*dependent) / scale);
        };
    return derive<T>(getter, setter);
  }

  /**
		 * Convenience method for deriving an attribute which covers one coefficient of this matrix
		 * @param U The type of the coefficient (usally Real or Complex)
		 * @param row The coefficients row coordinate
		 * @param column The coefficients column coordinate
		 * @return a new attribute whose value will always equal the value of the coefficient `this(row, column)`
		 * */
  template <class U, class V = T,
            std::enable_if_t<std::is_same_v<CPS::MatrixVar<U>, V>, bool> = true>
  AttributePointer<Attribute<U>>
  deriveCoeff(typename CPS::MatrixVar<U>::Index row,
              typename CPS::MatrixVar<U>::Index column)
  // requires std::same_as<T, CPS::MatrixVar<U>> //CPP20
  {
    typename AttributeUpdateTask<U, T>::Actor getter =
        [row, column](std::shared_ptr<U> &dependent,
                      Attribute<T>::Ptr dependency) {
          *dependent = (**dependency)(row, column);
        };
    typename AttributeUpdateTask<U, T>::Actor setter =
        [row, column](std::shared_ptr<U> &dependent,
                      Attribute<T>::Ptr dependency) {
          CPS::MatrixVar<U> currentValue = dependency->get();
          currentValue(row, column) = *dependent;
          dependency->set(currentValue);
        };
    return derive<U>(getter, setter);
  }
};

/**
	 * Class for static attributes. A static attribute's value can only ever by changed via the `get` and `set` methods (or the reference provided by `get`).
	 * Static attributes do not directly depend on any other attributes and currently cannot have any update tasks.
	 * */
template <class T>
class AttributeStatic : public Attribute<T>,
                        public SharedFactory<AttributeStatic<T>> {
  friend class SharedFactory<AttributeStatic<T>>;

public:
  AttributeStatic(T initialValue = T()) : Attribute<T>(initialValue) {}

  virtual void set(T value) override { *this->mData = value; };

  virtual T &get() override { return *this->mData; };

  virtual bool isStatic() const override { return true; }

  virtual void setReference(typename Attribute<T>::Ptr reference) override {
    throw TypeException();
  }

  virtual std::shared_ptr<T> asRawPointer() override { return this->mData; }

  virtual void appendDependencies(AttributeBase::Set *deps) override {
    deps->insert(this->shared_from_this());
  }
};

/**
	 * Class for dynamic attributes. A dynamic attribute has an internal value which can be updated by update tasks.
	 * */
template <class T>
class AttributeDynamic : public Attribute<T>,
                         public SharedFactory<AttributeDynamic<T>> {
  friend class SharedFactory<AttributeDynamic<T>>;

protected:
  std::vector<typename AttributeUpdateTaskBase<T>::Ptr> updateTasksOnce;
  std::vector<typename AttributeUpdateTaskBase<T>::Ptr> updateTasksOnGet;
  std::vector<typename AttributeUpdateTaskBase<T>::Ptr> updateTasksOnSet;

public:
  AttributeDynamic(T initialValue = T()) : Attribute<T>(initialValue) {}

  /**
		 * Allows for adding a new update task to this attribute.
		 * @param kind The kind of update task
		 * @param task The update task itself
		 * */
  void addTask(UpdateTaskKind kind,
               typename AttributeUpdateTaskBase<T>::Ptr task) {
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

  /**
		 * Removes all update tasks of a given kind from this attribute.
		 * @param kind The kind of tasks to remove
		 * */
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

  /**
		 * Remove all update tasks from this attribute, regardless of their kind.
		 * */
  void clearAllTasks() {
    updateTasksOnce.clear();
    updateTasksOnGet.clear();
    updateTasksOnSet.clear();
  }

  virtual void setReference(typename Attribute<T>::Ptr reference) override {
    typename AttributeUpdateTask<T, T>::Actor getter =
        [](std::shared_ptr<T> &dependent,
           typename Attribute<T>::Ptr dependency) {
          dependent = dependency->asRawPointer();
        };
    this->clearAllTasks();
    if (reference->isStatic()) {
      this->addTask(UpdateTaskKind::UPDATE_ONCE,
                    AttributeUpdateTask<T, T>::make(UpdateTaskKind::UPDATE_ONCE,
                                                    getter, reference));
    } else {
      this->addTask(UpdateTaskKind::UPDATE_ON_GET,
                    AttributeUpdateTask<T, T>::make(
                        UpdateTaskKind::UPDATE_ON_GET, getter, reference));
    }
  }

  virtual std::shared_ptr<T> asRawPointer() override {
    for (typename AttributeUpdateTaskBase<T>::Ptr task : updateTasksOnGet) {
      task->executeUpdate(this->mData);
    }
    return this->mData;
  }

  virtual void set(T value) override {
    *this->mData = value;
    for (typename AttributeUpdateTaskBase<T>::Ptr task : updateTasksOnSet) {
      task->executeUpdate(this->mData);
    }
  };

  virtual T &get() override {
    for (typename AttributeUpdateTaskBase<T>::Ptr task : updateTasksOnGet) {
      task->executeUpdate(this->mData);
    }
    return *this->mData;
  };

  virtual bool isStatic() const override { return false; }

  /**
		 * Implementation for dynamic attributes.This will recursively collect all attributes this attribute depends on, either in the UPDATE_ONCE or the UPDATE_ON_GET tasks.
		 * This is done by performing a Depth-First-Search on the dependency graph where the task dependencies of each attribute are the outgoing edges.
		 * The `deps` set contains all the nodes that have already been visited in the graph
		 * */
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

template <> String Attribute<Real>::toString();

template <> String Attribute<Complex>::toString();

template <> String Attribute<String>::toString();

} // namespace CPS

namespace std {
/**
	 * Struct for making the custom `AttributePointer` type hashable. This enables these pointers to be used in STL Maps.
	 * */
template <typename T> struct hash<CPS::AttributePointer<T>> {
  size_t operator()(CPS::AttributePointer<T> const &x) const {
    return std::hash<std::shared_ptr<T>>()(x.getPtr());
  }
};
} // namespace std
