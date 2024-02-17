#pragma once

namespace pf
{
	class State;
	using StatePtr = std::shared_ptr<State>;

	class State
	{
	public:
		State(int uid) : uid(uid) {}
		virtual float getCost() = 0;
		virtual std::vector<std::shared_ptr<State>> getNeighbors() = 0;

		bool operator==(const State& other) const { return uid == other.uid; }

	private:
		int uid;
	};
}
