#pragma once

#include <algorithm>

namespace pf
{
	float EuclideanDistance(float x1, float y1, float x2, float y2)
	{
		return static_cast<float>(sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2)));
	}

	float ManhattanDistance(float x1, float y1, float x2, float y2)
	{
		return static_cast<float>(abs(x2 - x1) + abs(y2 - y1));
	}

	template<typename T>
	class State
	{
	public:
		static size_t evaluationCount;

		virtual bool operator==(T& other) const = 0;
		virtual float getFCost() = 0;
		virtual float getGCost() = 0;
		virtual float getHCost() = 0;
		virtual bool isGoal() = 0;
		virtual std::vector<std::shared_ptr<T>> getNeighbours() = 0;
		virtual std::shared_ptr<T> getParent() const { return parent; }

	protected:
		State(std::shared_ptr<T> parent = nullptr) : parent(parent) {}
		std::shared_ptr<T> parent;
	};

	template<typename T>
	struct Path
	{
		bool found;
		std::vector<std::shared_ptr<T>> nodes;
		float cost;
	};

	template<typename T>
	Path<T> asPathfinding(std::shared_ptr<T> start, bool toLog = false)
	{
		static_assert(std::is_base_of<State<T>, T>::value, "T must be a subclass of State<T>.");
		T::evaluationCount++;

		// Start open set with start node
		// Changed from std::set to std::vector to ensure deterministic outcome
		std::vector<std::shared_ptr<T>> closedSet;//std::set<std::shared_ptr<T>> closedSet;
		std::vector<std::shared_ptr<T>> openSet;//std::set<std::shared_ptr<T>> openSet;
		openSet.push_back(start);//openSet.insert(start);

		// Until open set is empty
		while (!openSet.empty())
		{
			// Get node with lowest f cost
			std::shared_ptr<T> current = *openSet.begin();
			for (const auto& node : openSet)
			{
				if (node->getFCost() < current->getFCost())
				{
					current = node;
				}
			}

			// Found goal
			if (current->isGoal())
			{
				std::vector<std::shared_ptr<T>> path;
				float cost = 0.0f;

				std::shared_ptr<T> node = current;
				while (node != nullptr)
				{
					path.push_back(node);
					cost += node->getGCost();
					node = node->getParent();
				}
				std::reverse(path.begin(), path.end());

				return { true, path, cost };
			}

			openSet.erase(std::remove(openSet.begin(), openSet.end(), current), openSet.end());//openSet.erase(current);
			closedSet.push_back(current);//closedSet.insert(current);

			// Add neighbours to open if not in open or closed set
			for (std::shared_ptr<T> neighbour : current->getNeighbours())
			{
				if (std::find_if(closedSet.begin(), closedSet.end(), [&neighbour](const std::shared_ptr<T>& node) { return *node == *neighbour; }) != closedSet.end())
				{
					continue;
				}
				if (std::find_if(openSet.begin(), openSet.end(), [&neighbour](const std::shared_ptr<T>& node) { return *node == *neighbour; }) == openSet.end())
				{
					openSet.push_back(neighbour);//openSet.insert(neighbour);
				}
			}
		}

		return { false, {}, -1.0f };
	}
}
