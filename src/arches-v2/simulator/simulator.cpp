#include "simulator.hpp"

#include "units/unit-base.hpp"

namespace Arches {

	/**
	 * The following two functions add units to a "unit group" struct. 
	 * 
	 * TODO: figure out what the heck a unit group even does. it *seems* self-explanatory, but i feel that there may be more to it...
	 */
void Simulator::register_unit(Units::UnitBase * unit)
{
	unit->unit_id = _units.size();
	_units.push_back(unit);
	_unit_groups.back().end++;
	unit->simulator = this;
}

void Simulator::new_unit_group()
{
	_unit_groups.emplace_back(static_cast<uint>(_units.size()), static_cast<uint>(_units.size()));
}

#ifndef _DEBUG
#define USE_TBB		//TBB is Intel's OneAPI "Thread Building Blocks" library. There doesn't appear to be any alternate threading library in use anyway.
#endif

#ifdef USE_TBB
//tbb controled block ranges
//#define UNIT_LOOP tbb::parallel_for(tbb::blocked_range<uint>(0, _units.size()), [&](tbb::blocked_range<uint> r) { for(uint i = r.begin(); i < r.end(); ++i) {
//#define UNIT_LOOP_END }});

//custom block ranges
#define UNIT_LOOP tbb::parallel_for(tbb::blocked_range<uint>(0, _unit_groups.size()), [&](tbb::blocked_range<uint> r) { for(uint j = r.begin(); j < r.end(); ++j) { for(uint i = _unit_groups[j].start; i < _unit_groups[j].end; ++i) {
#define UNIT_LOOP_END }}});

#else
#define UNIT_LOOP for(uint i = 0; i < _units.size(); ++i) {
#define UNIT_LOOP_END }
#endif

void Simulator::execute(uint delta, std::function<void()> interval_logger)
{
#ifdef USE_TBB
	tbb::task_arena::constraints arena_constraints;
	arena_constraints.set_max_concurrency(8);
	//arena_constraints.set_max_threads_per_core(1);
	//arena_constraints.set_core_type(tbb::info::core_types().back());

	tbb::task_arena(arena_constraints).execute([&]
	{
#endif
		UNIT_LOOP
			_units[i]->reset();
		UNIT_LOOP_END

		do
		{
			UNIT_LOOP
				_units[i]->clock_rise();
			UNIT_LOOP_END

			UNIT_LOOP
				_units[i]->clock_fall();
			UNIT_LOOP_END

			current_cycle++;
			if(delta != 0 && current_cycle % delta == 0)
				interval_logger();
		}
		while(units_executing > 0);

#ifdef USE_TBB
	});
#endif
}

}