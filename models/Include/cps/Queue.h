/** Bounded MPMC queue
 *
 * Based on Dmitry Vyukov#s Bounded MPMC queue:
 *	 http://www.1024cores.net/home/lock-free-algorithms/queues/bounded-mpmc-queue
 *
 * @file
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2018, Institute for Automation of Complex Power Systems, EONERC
 *
 * DPsim
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.	If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#pragma once

#include <atomic>

namespace CPS {

	template<typename T>
	class Queue {

	public:
		Queue(size_t buffersize) :
			buffer(new Cell[buffersize]),
			buffermask(buffersize - 1)
		{
			assert((buffersize >= 2) && ((buffersize & (buffersize - 1)) == 0));

			for (size_t i = 0; i != buffersize; i += 1) {
				buffer[i].sequence.store(i, std::memory_order_relaxed);
			}

			enqueue_pos.store(0, std::memory_order_relaxed);
			dequeue_pos.store(0, std::memory_order_relaxed);
		}

		~Queue() {
			delete[] buffer;
		}

		bool enqueue(T const &data) {
			Cell* cell;
			size_t pos = enqueue_pos.load(std::memory_order_relaxed);

			for (;;) {
				cell = &buffer[pos & buffermask];

				size_t seq = cell->sequence.load(std::memory_order_acquire);
				intptr_t dif = (intptr_t) seq - (intptr_t) pos;

				if (dif == 0) {
					if (enqueue_pos.compare_exchange_weak(pos, pos + 1, std::memory_order_relaxed)) {
						break;
					}
				}
				else if (dif < 0) {
					return false;
				}
				else {
					pos = enqueue_pos.load(std::memory_order_relaxed);
				}
			}

			cell->data = data;
			cell->sequence.store(pos + 1, std::memory_order_release);

			return true;
		}

		bool dequeue(T &data) {
			Cell* cell;
			size_t pos = dequeue_pos.load(std::memory_order_relaxed);

			for (;;) {
				cell = &buffer[pos & buffermask];

				size_t seq = cell->sequence.load(std::memory_order_acquire);
				intptr_t dif = (intptr_t)seq - (intptr_t)(pos + 1);

				if (dif == 0) {
					if (dequeue_pos.compare_exchange_weak(pos, pos + 1, std::memory_order_relaxed)) {
						break;
					}
				}
				else if (dif < 0) {
					return false;
				}
				else {
					pos = dequeue_pos.load(std::memory_order_relaxed);
				}
			}

			data = cell->data;
			cell->sequence.store(pos + buffermask + 1, std::memory_order_release);

			return true;
		}

	private:
		struct Cell {
			std::atomic<size_t> sequence;
			T data;
		};

		static size_t const cacheline_size = 64;
		typedef char cacheline_pad_t[cacheline_size];

		// Note: only change the following members if you really know what you are doing!
		cacheline_pad_t	pad0;
		Cell* const buffer;
		size_t const buffermask;
		cacheline_pad_t pad1;
		std::atomic<size_t> enqueue_pos;
		cacheline_pad_t pad2;
		std::atomic<size_t> dequeue_pos;
		cacheline_pad_t pad3;

		Queue(Queue const&);
		void operator=(Queue const&);
	};
}
