#pragma once

#include <algorithm>
#include <queue>
#include <unordered_set>
#include <deque>

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

	// T = State { State }
	// D = Data { hash, ... }
	template<typename T, typename D>
	class State
	{
	public:
		static size_t evaluationCount;

		State(D data, T* parent)
			: data(data), parent(parent), dataHash(data.getHash())
		{}

		bool operator==(T& other) const { dataHash = other.dataHash; }

		virtual bool isGoal() = 0;
		virtual float getFCost() = 0;
		virtual float getGCost() = 0;
		virtual float getHCost() = 0;
		virtual std::vector<D*> getNeighbours() = 0;

		const D& getData() const { return data; }
		D moveData() { return std::move(data); }
		size_t getDataHash() const { return dataHash; }
		T* getParent() const { return parent; }

	protected:
		D data;
		size_t dataHash;
		T* parent;
	};

	template<typename D>
	struct Path
	{
		bool found;
		std::vector<D> nodes;
		float cost;
	};

	struct CompareNodeByFCost
	{
		template<typename T>
		bool operator()(T* a, T* b) const
		{
			return a->getFCost() > b->getFCost(); // Assuming lower f cost is better
		}
	};

	template<typename T, typename D>
	std::shared_ptr<Path<D>> asPathfinding(T* start, bool toLog = false)
	{
		T::evaluationCount++;

		// Start open set with start node
		std::priority_queue<T*, std::vector<T*>, CompareNodeByFCost> openSet;
		std::unordered_set<size_t> closedHashSet;
		std::unordered_set<size_t> openHashSet;
		std::deque<T*> allNodes;

		allNodes.push_back(start);
		openSet.push(start);
		openHashSet.insert(start->getDataHash());

		// Until open set is empty
		while (!openSet.empty())
		{
			// Get node with lowest f cost
			T* current = openSet.top();
			openSet.pop();

			// Found goal
			if (current->isGoal())
			{
				std::shared_ptr<Path<D>> path = std::make_shared<Path<D>>();

				T* node = current;
				while (node != nullptr)
				{
					path->nodes.push_back(node->moveData());
					path->cost += node->getGCost();
					node = node->getParent();
				}
				std::reverse(path->nodes.begin(), path->nodes.end());
				path->found = true;

				// Cleanup and return with path
				for (T* node : allNodes) delete node;
				return path;
			}

			// Delete current and remove from open set
			openHashSet.erase(current->getDataHash());
			closedHashSet.insert(current->getDataHash());

			// Add neighbours to open if not in open or closed set
			const auto neighbours = current->getNeighbours();
			for (const auto& neighbourDataPtr : neighbours)
			{
				const size_t neighbourHash = neighbourDataPtr->getHash();

				if (closedHashSet.find(neighbourHash) != closedHashSet.end()) continue;
				if (openHashSet.find(neighbourHash) != openHashSet.end()) continue;

				T* neighbour = new T(*neighbourDataPtr, current);
				allNodes.push_back(neighbour);
				openSet.push(neighbour);
				openHashSet.insert(neighbour->getDataHash());
			}
		}

		// Cleanup and return without path
		for (T* node : allNodes) delete node;
		std::shared_ptr<Path<D>> path = std::make_shared<Path<D>>();
		path->found = false;
		return path;
	}
}
