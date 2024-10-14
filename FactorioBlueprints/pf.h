#pragma once

#include <algorithm>
#include <queue>
#include <unordered_set>
#include <deque>
#include "log.h"
#include "macros.h"

namespace pf
{
	inline float EuclideanDistance(float x1, float y1, float x2, float y2)
	{
		return static_cast<float>(sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2)));
	}

	inline float ManhattanDistance(float x1, float y1, float x2, float y2)
	{
		return static_cast<float>(abs(x2 - x1) + abs(y2 - y1));
	}

	template<typename T, typename D>
	class State
	{
	public:
		State(D data, T* parent)
			: data(std::move(data)), parent(parent), hash(data.getHash())
		{}
		virtual bool isGoal() = 0;
		virtual float getFCost() = 0;
		virtual float getGCost() = 0;
		virtual float getHCost() = 0;
		virtual std::vector<D*> getNeighbours() = 0;
		const D& getData() const { return data; }
		T* getParent() const { return parent; }
		size_t getHash() const { return hash; }
		D moveData() { return std::move(data); }

	protected:
		D data;
		T* parent;
		size_t hash;
	};

	template<typename D>
	struct Path
	{
		bool found = false;
		std::vector<D> nodes;
		float cost = -1;
	};

	struct CompareNodeByFCost
	{
		template<typename T>
		bool operator()(T* a, T* b) const
		{
			// Assuming lower f cost is better
			return a->getFCost() > b->getFCost();
		}
	};

	template<typename T, typename D>
	Path<D> asPathfinding(T* start)
	{
		std::priority_queue<T*, std::vector<T*>, CompareNodeByFCost> openSet;
		std::unordered_set<size_t> closedSetHash;
		std::unordered_set<size_t> openSetHash;
		std::deque<T*> allNodes;

		allNodes.push_back(start);
		openSet.push(start);
		openSetHash.insert(start->getHash());
		LOG(PATHFINDING, "Start, G Cost: " << start->getGCost() << "\n");

		// Until open set is empty
		while (!openSet.empty())
		{
			// Get node with lowest f cost
			T* current = openSet.top();
			LOG(PATHFINDING, "Current, G Cost: " << current->getGCost() << ", H Cost: " << current->getHCost() << ", F Cost: " << current->getFCost() << "\n");

			// Found goal so extract path and cleanup nodes
			if (current->isGoal())
			{
				Path<D> path = Path<D>();

				T* node = current;
				while (node != nullptr)
				{
					path.nodes.push_back(node->moveData());
					path.cost += node->getGCost();
					node = node->getParent();
				}
				for (T* node : allNodes) delete node;

				LOG(PATHFINDING, "Path found, cost: " << path.cost << "\n");
				std::reverse(path.nodes.begin(), path.nodes.end());
				path.found = true;
				return path;
			}

			// Remove from open and add to closed
			openSet.pop();
			openSetHash.erase(current->getHash());
			closedSetHash.insert(current->getHash());

			// Add neighbours to open if not in open or closed set
			const auto neighbours = current->getNeighbours();
			for (const auto& neighbourDataPtr : neighbours)
			{
				const size_t neighbourHash = neighbourDataPtr->getHash();

				if (closedSetHash.find(neighbourHash) != closedSetHash.end()) continue;
				if (openSetHash.find(neighbourHash) != openSetHash.end()) continue;

				T* neighbour = new T(*neighbourDataPtr, current);
				allNodes.push_back(neighbour);
				openSet.push(neighbour);
				openSetHash.insert(neighbour->getHash());
			}
		}

		// Cleanup and return without path
		for (T* node : allNodes) delete node;
		Path<D> path = Path<D>();
		path.found = false;
		return path;
	}
}
