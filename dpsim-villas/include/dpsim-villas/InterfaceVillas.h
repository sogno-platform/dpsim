// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <dpsim-models/PtrFactory.h>
#include <dpsim/InterfaceManager.h>

#include <villas/node.hpp>
#include <villas/exceptions.hpp>
#include <villas/memory.hpp>
#include <villas/kernel/rt.hpp>
#include <villas/pool.hpp>
#include <villas/sample.hpp>
#include <villas/signal.hpp>
#include <villas/signal_list.hpp>

using namespace villas;

namespace DPsim {
	class InterfaceVillas :
		public Interface,
		public SharedFactory<InterfaceVillas> {

	public:
		typedef std::shared_ptr<InterfaceVillas> Ptr;
		typedef struct node::Sample Sample;

		static UInt villasPriority;
		static UInt villasAffinity;
		static UInt villasHugePages;

	protected:
		static Bool villasInitialized;

		// Using std::function / lambda makes the other template code nicer, but from
		// the outside, only the attribute-based functions should be used to
		// guarantee proper scheduling
		void addImport(std::function<CPS::AttributeBase::Ptr(Sample*)> l) { mImports.push_back(l); }
		void addExport(std::function<void(CPS::AttributeBase::Ptr, Sample*)> l) { mExports.push_back(l); }

		std::vector<std::function<CPS::AttributeBase::Ptr(Sample*)>> mImports;
		std::vector<std::function<void(CPS::AttributeBase::Ptr, Sample*)>> mExports;
		
		//Villas node to send / receive data to / from
		String mNodeConfig;
		node::Node* mNode;

		int mQueueLength;
		int mSampleLength;
		node::Pool mSamplePool;

		Sample *mLastSample;
		int mSequence;

		std::map<int, node::Signal::Ptr> mExportSignals;
		std::map<int, node::Signal::Ptr> mImportSignals;

	public:

		InterfaceVillas(const String &nodeConfig, UInt queueLenght = 512, UInt sampleLenght = 64);

		virtual void open() override;
		virtual void close() override;
		
		CPS::Attribute<Int>::Ptr importInt(UInt idx);
		CPS::Attribute<Real>::Ptr importReal(UInt idx);
		CPS::Attribute<Bool>::Ptr importBool(UInt idx);
		CPS::Attribute<Complex>::Ptr importComplex(UInt idx);
		CPS::Attribute<Complex>::Ptr importComplexMagPhase(UInt idx);

		void exportInt(CPS::Attribute<Int>::Ptr attr, UInt idx, const std::string &name="", const std::string &unit="");
		void exportReal(CPS::Attribute<Real>::Ptr attr, UInt idx, const std::string &name="", const std::string &unit="");
		void exportBool(CPS::Attribute<Bool>::Ptr attr, UInt idx, const std::string &name="", const std::string &unit="");
		void exportComplex(CPS::Attribute<Complex>::Ptr attr, UInt idx, const std::string &name="", const std::string &unit="");
	
		virtual void readValuesFromEnv(CPS::AttributeBase::List& updatedAttrs) override;
		virtual void writeValuesToEnv(CPS::AttributeBase::List& updatedAttrs) override;
	
	private:
		void prepareNode();
		void setupNodeSignals();
		void initVillas();
	};
}

