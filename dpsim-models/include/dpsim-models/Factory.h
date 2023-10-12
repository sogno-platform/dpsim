#include <dpsim-models/Logger.h>
#include <dpsim-models/Definitions.h>
#include <map>

#include <dpsim-models/Base/Base_ReducedOrderSynchronGenerator.h>
#include <dpsim-models/SP/SP_Ph1_ReducedOrderSynchronGeneratorVBR.h>
#include <dpsim-models/SP/SP_Ph1_SynchronGenerator3OrderVBR.h>
#include <dpsim-models/SP/SP_Ph1_SynchronGenerator4OrderVBR.h>
#include <dpsim-models/SP/SP_Ph1_SynchronGenerator6aOrderVBR.h>
#include <dpsim-models/SP/SP_Ph1_SynchronGenerator6bOrderVBR.h>
#include <dpsim-models/DP/DP_Ph1_ReducedOrderSynchronGeneratorVBR.h>
#include <dpsim-models/DP/DP_Ph1_SynchronGenerator3OrderVBR.h>
#include <dpsim-models/DP/DP_Ph1_SynchronGenerator4OrderVBR.h>
#include <dpsim-models/DP/DP_Ph1_SynchronGenerator4OrderPCM.h>
#include <dpsim-models/DP/DP_Ph1_SynchronGenerator4OrderTPM.h>
#include <dpsim-models/DP/DP_Ph1_SynchronGenerator6aOrderVBR.h>
#include <dpsim-models/DP/DP_Ph1_SynchronGenerator6bOrderVBR.h>
#include <dpsim-models/DP/DP_Ph1_SynchronGenerator6OrderPCM.h>
#include <dpsim-models/EMT/EMT_Ph3_ReducedOrderSynchronGeneratorVBR.h>
#include <dpsim-models/EMT/EMT_Ph3_SynchronGenerator3OrderVBR.h>
#include <dpsim-models/EMT/EMT_Ph3_SynchronGenerator4OrderVBR.h>
#include <dpsim-models/EMT/EMT_Ph3_SynchronGenerator6aOrderVBR.h>
#include <dpsim-models/EMT/EMT_Ph3_SynchronGenerator6bOrderVBR.h>
#include <dpsim-models/Base/Base_Exciter.h>
#include <dpsim-models/Signal/ExciterDC1.h>
#include <dpsim-models/Signal/ExciterDC1Simp.h>
#include <dpsim-models/Signal/ExciterST1Simp.h>

#pragma once


template <class BaseClass>
class Creator {
    
    public:
    virtual ~Creator(){}

    virtual std::shared_ptr<BaseClass> Create(const std::string & name, CPS::Logger::Level logLevel = CPS::Logger::Level::debug) = 0;
};

template <class DerivedClass, class BaseClass>
class DerivedCreator : public Creator<BaseClass> {

    public:
    std::shared_ptr<BaseClass> Create(const std::string & name, CPS::Logger::Level logLevel = CPS::Logger::Level::debug) {
        return std::shared_ptr<BaseClass>(new DerivedClass(name, logLevel));
    }
};

template <class BaseClass>
class Factory {
    public:
        static Factory & get() {
            static Factory instance;
            return instance;
        }

        std::vector<std::string> getItems() {            
            std::vector<std::string> items;
            for (auto g : functionMap) {
                items.push_back(g.first);
            }

            return items;
        }

        std::shared_ptr<BaseClass> create(
            std::string type, const std::string & name, 
            CPS::Logger::Level logLevel = CPS::Logger::Level::debug) {
            
            auto it = functionMap.find(type);
            if (it != functionMap.end())
                return it->second->Create(name, logLevel);
            else
            	throw CPS::SystemError("Unsupported type '" + type + "'!");	
        }

        void registerExciter(
            const std::string& type, 
            Creator<BaseClass>* Fn) {
                functionMap[type] = Fn;
        }

    private:
        Factory() { }
        Factory(const Factory&);
        ~Factory() {
            auto i = functionMap.begin();
            while (i != functionMap.end()) {
                delete (*i).second;
                ++i;
            }
        }

        std::map<std::string, Creator<BaseClass>*> functionMap;
};

template <class BaseClass>
class FactoryRegistration {
    public:
        FactoryRegistration(std::string type, Creator<BaseClass>* Fn) {
            Factory<BaseClass>::get().registerExciter(type, Fn);
		}
};

namespace ExciterFactory {
    void registerExciters() {
        FactoryRegistration<CPS::Base::Exciter> _ExciterDC1("DC1", new DerivedCreator<CPS::Signal::ExciterDC1, CPS::Base::Exciter>);
        FactoryRegistration<CPS::Base::Exciter> _ExciterDC1Simp("DC1Simp", new DerivedCreator<CPS::Signal::ExciterDC1Simp, CPS::Base::Exciter>);
        FactoryRegistration<CPS::Base::Exciter> _ExciterST1Simp("ST1", new DerivedCreator<CPS::Signal::ExciterST1Simp, CPS::Base::Exciter>);
    }
}

namespace SynchronGeneratorFactory {
namespace SP {
namespace Ph1 {
    void registerSynchronGenerators() {
        FactoryRegistration<CPS::SP::Ph1::ReducedOrderSynchronGeneratorVBR> _3OrderSP("3", new DerivedCreator<CPS::SP::Ph1::SynchronGenerator3OrderVBR, CPS::SP::Ph1::ReducedOrderSynchronGeneratorVBR>);
        FactoryRegistration<CPS::SP::Ph1::ReducedOrderSynchronGeneratorVBR> _4OrderSP("4", new DerivedCreator<CPS::SP::Ph1::SynchronGenerator4OrderVBR, CPS::SP::Ph1::ReducedOrderSynchronGeneratorVBR>);
        FactoryRegistration<CPS::SP::Ph1::ReducedOrderSynchronGeneratorVBR> _5OrderSP("5", new DerivedCreator<CPS::SP::Ph1::SynchronGenerator5OrderVBR, CPS::SP::Ph1::ReducedOrderSynchronGeneratorVBR>);
        FactoryRegistration<CPS::SP::Ph1::ReducedOrderSynchronGeneratorVBR> _6aOrderSP("6a", new DerivedCreator<CPS::SP::Ph1::SynchronGenerator6aOrderVBR, CPS::SP::Ph1::ReducedOrderSynchronGeneratorVBR>);
        FactoryRegistration<CPS::SP::Ph1::ReducedOrderSynchronGeneratorVBR> _6bOrderSP("6b", new DerivedCreator<CPS::SP::Ph1::SynchronGenerator6bOrderVBR, CPS::SP::Ph1::ReducedOrderSynchronGeneratorVBR>);
    }
}
}

namespace DP {
namespace Ph1 {
    void registerSynchronGenerators() {
        FactoryRegistration<CPS::DP::Ph1::ReducedOrderSynchronGeneratorVBR> _3OrderSP("3", new DerivedCreator<CPS::DP::Ph1::SynchronGenerator3OrderVBR, CPS::DP::Ph1::ReducedOrderSynchronGeneratorVBR>);
        FactoryRegistration<CPS::DP::Ph1::ReducedOrderSynchronGeneratorVBR> _4OrderSP("4", new DerivedCreator<CPS::DP::Ph1::SynchronGenerator4OrderVBR, CPS::DP::Ph1::ReducedOrderSynchronGeneratorVBR>);
        FactoryRegistration<CPS::Base::ReducedOrderSynchronGenerator<CPS::Complex>> _4OrderDPIter("4PCM", new DerivedCreator<CPS::DP::Ph1::SynchronGenerator4OrderPCM, CPS::Base::ReducedOrderSynchronGenerator<CPS::Complex>>);
        FactoryRegistration<CPS::Base::ReducedOrderSynchronGenerator<CPS::Complex>> _4OrderDPTPM("4TPM", new DerivedCreator<CPS::DP::Ph1::SynchronGenerator4OrderTPM, CPS::Base::ReducedOrderSynchronGenerator<CPS::Complex>>);
        FactoryRegistration<CPS::DP::Ph1::ReducedOrderSynchronGeneratorVBR> _6aOrderSP("6a", new DerivedCreator<CPS::DP::Ph1::SynchronGenerator6aOrderVBR, CPS::DP::Ph1::ReducedOrderSynchronGeneratorVBR>);
        FactoryRegistration<CPS::DP::Ph1::ReducedOrderSynchronGeneratorVBR> _6bOrderSP("6b", new DerivedCreator<CPS::DP::Ph1::SynchronGenerator6bOrderVBR, CPS::DP::Ph1::ReducedOrderSynchronGeneratorVBR>);
        FactoryRegistration<CPS::Base::ReducedOrderSynchronGenerator<CPS::Complex>> _6OrderDPIter("6PCM", new DerivedCreator<CPS::DP::Ph1::SynchronGenerator6OrderPCM, CPS::Base::ReducedOrderSynchronGenerator<CPS::Complex>>);
    }
}
}

namespace EMT {
namespace Ph3 {
    void registerSynchronGenerators() {
        FactoryRegistration<CPS::EMT::Ph3::ReducedOrderSynchronGeneratorVBR> _3OrderSP("3", new DerivedCreator<CPS::EMT::Ph3::SynchronGenerator3OrderVBR, CPS::EMT::Ph3::ReducedOrderSynchronGeneratorVBR>);
        FactoryRegistration<CPS::EMT::Ph3::ReducedOrderSynchronGeneratorVBR> _4OrderSP("4", new DerivedCreator<CPS::EMT::Ph3::SynchronGenerator4OrderVBR, CPS::EMT::Ph3::ReducedOrderSynchronGeneratorVBR>);
        FactoryRegistration<CPS::EMT::Ph3::ReducedOrderSynchronGeneratorVBR> _6aOrderSP("6a", new DerivedCreator<CPS::EMT::Ph3::SynchronGenerator6aOrderVBR, CPS::EMT::Ph3::ReducedOrderSynchronGeneratorVBR>);
        FactoryRegistration<CPS::EMT::Ph3::ReducedOrderSynchronGeneratorVBR> _6bOrderSP("6b", new DerivedCreator<CPS::EMT::Ph3::SynchronGenerator6bOrderVBR, CPS::EMT::Ph3::ReducedOrderSynchronGeneratorVBR>);
    }
}
}

}