#pragma once

#include <cstdint>
#include <iostream>
#include <set>
#include <map>
#include <vector>
#include <unordered_map>
#include <mutex>
#include "types.h"
#include "pf.h"

namespace impl
{
	struct CATEntry
	{
		std::set<size_t> conflictHashes;
	};

	inline CATEntry calculateCATEntry(Coordinate coordinate, BeltType type, Direction direction)
	{
		bool isAboveGround = false;
		bool isUndergroundNS = false;
		bool isUndergroundEW = false;
		if (type == BeltType::Inserter || type == BeltType::Conveyor || type == BeltType::UndergroundEntrance || type == BeltType::UndergroundExit)
		{
			isAboveGround = true;
		}
		if (type == BeltType::Underground || type == BeltType::UndergroundEntrance || type == BeltType::UndergroundExit)
		{
			if (direction == Direction::N || direction == Direction::S)
			{
				isUndergroundNS = true;
			}
			else
			{
				isUndergroundEW = true;
			}
		}

		CATEntry entry;
		std::hash<size_t> intHasher;
		size_t coordinateHash = intHasher(coordinate.x) ^ (intHasher(coordinate.y) << 1);
		if (isAboveGround) entry.conflictHashes.insert(coordinateHash ^ (intHasher(1) << 2));
		if (isUndergroundNS) entry.conflictHashes.insert(coordinateHash ^ (intHasher(2) << 2));
		if (isUndergroundEW) entry.conflictHashes.insert(coordinateHash ^ (intHasher(3) << 2));
		return entry;
	}

	inline bool checkCATEntryConflict(const CATEntry& a, const CATEntry& b)
	{
		// Check if any of the conflict hashes are the same
		for (size_t hash : a.conflictHashes)
		{
			if (b.conflictHashes.find(hash) != b.conflictHashes.end())
			{
				return true;
			}
		}
		return false;
	}

	class CAT
	{
	public:
		CAT();
		void addConflict(size_t pathIndex, const CATEntry& entry);
		void removeConflict(size_t pathIndex);
		bool checkConflict(const CATEntry& entry) const;
		std::pair<CATEntry, std::set<size_t>> getConflictingPaths() const;

	private:
		std::map<size_t, std::set<size_t>> conflictTable;
	};

	struct PathfindingGoal
	{
	public:
		PathfindingGoal() : isValid(false) {}

		PathfindingGoal(Coordinate coordinate, uint8_t typeFlags, uint8_t directionFlags)
			: coordinate(coordinate), typeFlags(typeFlags), directionFlags(directionFlags), isValid(true)
		{}

		Coordinate coordinate;
		uint8_t typeFlags = 0;
		uint8_t directionFlags = 0;
		bool isValid = false;
	};

	struct PathfindingData
	{
		Coordinate coordinate;
		BeltType type;
		Direction direction;

		inline size_t getHash() const
		{
			std::hash<size_t> hasher;
			return hasher(coordinate.x) ^ (hasher(coordinate.y) << 1) ^ (hasher(static_cast<int>(type)) << 2) ^ (hasher(static_cast<int>(direction)) << 3);
		}

		inline CATEntry getCATEntry() const
		{
			return calculateCATEntry(coordinate, type, direction);
		}

		inline void print() const
		{
			std::string typeStr = "Unknown";
			switch (type)
			{
			case BeltType::None: typeStr = "None"; break;
			case BeltType::Conveyor: typeStr = "Conveyor"; break;
			case BeltType::UndergroundEntrance: typeStr = "UndergroundEntrance"; break;
			case BeltType::Underground: typeStr = "Underground"; break;
			case BeltType::UndergroundExit: typeStr = "UndergroundExit"; break;
			case BeltType::Inserter: typeStr = "Inserter"; break;
			}

			std::cout << "[ (" << coordinate.x << ", " << coordinate.y << ") " << dirString(direction) << " | " << typeStr << " ]" << std::endl;
		}

		inline bool isAboveGround() const
		{
			return type == BeltType::Conveyor || type == BeltType::UndergroundEntrance || type == BeltType::UndergroundExit;
		}
	};

	class PathfindingState : public pf::State<PathfindingState, PathfindingData>
	{
	public:
		PathfindingState(const std::vector<std::vector<bool>>& blockedGrid, const std::vector<CATEntry>& constraints, PathfindingGoal goal, PathfindingData data);
		PathfindingState(PathfindingData data, PathfindingState* parent);
		bool isGoal() override;
		float getFCost() override;
		float getGCost() override;
		float getHCost() override;
		std::vector<PathfindingData*> getNeighbours() override;
		const PathfindingGoal& getGoal() const;
		bool conflictsWithConstraints() const;

	private:
		const std::vector<std::vector<bool>>& blockedGrid;
		const std::vector<CATEntry>& constraints;
		PathfindingGoal goal;
		bool isFitnessCalculated = false;
		float fCost = 0.0f, gCost = 0.0f, hCost = 0.0f;
		void calculateCosts();
		void calculateNeighbourCache();

	private:
		static std::unordered_map<size_t, std::vector<std::pair<PathfindingData, CATEntry>>> neighbourCache;
	};

	struct ItemEndpoint
	{
		int item;
		float rate;
		bool isSource;
		Coordinate coordinate;
	};

	struct ConcreteEndpoint
	{
		enum class Type { ITEM, PATH };
		Type type;
		size_t index;
	};

	struct PathConfig
	{
		ConcreteEndpoint source;
		ConcreteEndpoint destination;
		size_t pathGroup;
		std::vector<size_t> dependantPaths = std::vector<size_t>();
	};

	class PlacementPathfinder
	{
	public:
		struct CTConflict
		{
			bool isConflict;
			size_t pathA;
			size_t pathB;
			CATEntry catEntry;
		};

		struct CTNode
		{
			bool isValid = true;
			std::vector<size_t> pathsToCalculate{};
			std::map<size_t, std::vector<CATEntry>> constraints{};
			std::map<size_t, std::shared_ptr<pf::Path<PathfindingData>>> solution{};
			CAT cat;
			CTConflict firstConflict{};
			float cost = 0.0f;
		};

		PlacementPathfinder(const std::vector<std::vector<bool>>& blockedGrid, const std::vector<ItemEndpoint>& itemEndpoints);
		float getFitness();
		size_t getPathsFound() const;
		void print();

	private:
		struct CompareCTNodeByCost
		{
			bool operator()(const std::shared_ptr<CTNode>& a, const std::shared_ptr<CTNode>& b) const
			{
				return a->cost > b->cost;
			}
		};

		const std::vector<std::vector<bool>>& blockedGrid;
		const std::vector<ItemEndpoint>& itemEndpoints;
		size_t currentPathGroup = 0;
		std::map<size_t, float> pathGroupSpareRates;
		std::vector<PathConfig> pathConfigs;
		std::shared_ptr<CTNode> solutionNode;
		bool solutionNodeFound = false;
		bool isFitnessCalculated = false;
		float fitness = 0.0f;

		void solve();
		void preprocessPaths();
		void performPathfinding();
		void processCTNode(std::shared_ptr<CTNode> node);
		PathfindingState* initPathNodeWithContext(size_t pathIndex, const std::shared_ptr<CTNode>& node);
		std::tuple<bool, Coordinate, Direction> findPathEdgeWithContext(const std::shared_ptr<pf::Path<PathfindingData>>& path, const Coordinate& target, const std::shared_ptr<CTNode>& node, const std::vector<CATEntry>& constraints);
	};
}
