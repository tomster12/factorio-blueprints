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
		virtual size_t getHash() const = 0;

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
	std::shared_ptr<Path<T>> asPathfinding(std::shared_ptr<T> start, bool toLog = false)
	{
		static_assert(std::is_base_of<State<T>, T>::value, "T must be a subclass of State<T>.");
		T::evaluationCount++;

		// Start open set with start node
		// Vector used so it is deterministic
		std::vector<size_t> closedSet;
		std::vector<std::shared_ptr<T>> openSet;
		openSet.push_back(start);

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
				std::shared_ptr<Path<T>> pathPtr = std::make_shared<Path<T>>();
				pathPtr->found = true;

				std::shared_ptr<T> node = current;
				while (node != nullptr)
				{
					pathPtr->nodes.push_back(node);
					pathPtr->cost += node->getGCost();
					node = node->getParent();
				}
				std::reverse(pathPtr->nodes.begin(), pathPtr->nodes.end());

				return pathPtr;
			}

			openSet.erase(std::remove(openSet.begin(), openSet.end(), current), openSet.end());
			closedSet.push_back(current->getHash());

			// Add neighbours to open if not in open or closed set
			for (std::shared_ptr<T> neighbour : current->getNeighbours())
			{
				size_t neighbourHash = neighbour->getHash();
				if (std::find_if(closedSet.begin(), closedSet.end(), [&neighbourHash](size_t hash) { return hash == neighbourHash; }) != closedSet.end())
				{
					continue;
				}
				if (std::find_if(openSet.begin(), openSet.end(), [&neighbour](const std::shared_ptr<T>& node) { return *node == *neighbour; }) == openSet.end())
				{
					openSet.push_back(neighbour);
				}
			}
		}

		std::shared_ptr<Path<T>> pathPtr = std::make_shared<Path<T>>();
		pathPtr->found = false;
		return pathPtr;
	}
}
