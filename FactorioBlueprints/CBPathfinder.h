#pragma once

#include <cstdint>
#include <iostream>
#include <set>
#include <map>
#include <vector>
#include <unordered_map>
#include "types.h"
#include "pf.h"

struct PFGoal
{
public:
	PFGoal() : isValid(false) {}

	PFGoal(Coordinate coordinate, uint8_t typeFlags, uint8_t directionFlags)
		: coordinate(coordinate), typeFlags(typeFlags), directionFlags(directionFlags), isValid(true)
	{}

	Coordinate coordinate;
	uint8_t typeFlags = 0;
	uint8_t directionFlags = 0;
	bool isValid = false;
};

struct PFData
{
	Coordinate coordinate;
	BeltType type;
	Direction direction;

	size_t getHash() const
	{
		std::hash<size_t> hasher;
		return hasher(coordinate.x) ^ (hasher(coordinate.y) << 1) ^ (hasher(static_cast<int>(type)) << 2) ^ (hasher(static_cast<int>(direction)) << 3);
	}

	void print() const
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

	bool isAboveGround() const
	{
		return type == BeltType::Conveyor || type == BeltType::UndergroundEntrance || type == BeltType::UndergroundExit;
	}
};

struct CATEntry
{
	size_t coordinateHash;
	size_t conflictFlags;
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

inline CATEntry calculateCATEntry(Coordinate coordinate, BeltType type, Direction direction)
{
	// Calculate coordinate hash
	std::hash<size_t> hasher;
	size_t coordinateHash = hasher(coordinate.x) ^ (hasher(coordinate.y) << 1);

	// Calculate conflict group
	size_t conflictFlags = 0;
	if (type == BeltType::Inserter || type == BeltType::Conveyor || type == BeltType::UndergroundEntrance || type == BeltType::UndergroundExit)
	{
		conflictFlags |= 0b001;
	}

	if (type == BeltType::Underground || type == BeltType::UndergroundEntrance || type == BeltType::UndergroundExit)
	{
		if (direction == Direction::N || direction == Direction::S)
		{
			conflictFlags |= 0b010;
		}
		else conflictFlags |= 0b100;
	}

	// Return final entry
	return { coordinateHash, conflictFlags };
}

inline bool checkCATEntryConflict(const CATEntry& a, const CATEntry& b)
{
	bool coordMatch = a.coordinateHash == b.coordinateHash;
	bool setOverlap = (a.conflictFlags & b.conflictFlags) != 0;
	return coordMatch && setOverlap;
}

class ConflictAvoidanceTable
{
public:
	ConflictAvoidanceTable();
	ConflictAvoidanceTable(const ConflictAvoidanceTable& other);
	void addPath(size_t pathIndex, const CATEntry& entry);
	void removePath(size_t pathIndex);
	bool checkConflict(const CATEntry& entry) const;
	std::pair<CATEntry, std::set<size_t>> getConflictingPaths() const;

private:
	std::map<size_t, std::vector<std::set<size_t>>> table;
};

class PFState : public pf::State<PFState, PFData>
{
public:
	PFState(const std::vector<std::vector<bool>>& blockedGrid, const std::vector<CATEntry>& constraints, PFGoal goal, PFData data);
	PFState(PFData data, PFState* parent);
	bool isGoal() override;
	float getFCost() override;
	float getGCost() override;
	float getHCost() override;
	std::vector<PFData*> getNeighbours() override;
	const PFGoal& getGoal() const;
	bool conflictsWithConstraints() const;

private:
	static std::unordered_map<size_t, std::vector<std::pair<PFData, CATEntry>>> neighbourCache;
	const std::vector<std::vector<bool>>& blockedGrid;
	const std::vector<CATEntry>& constraints;
	PFGoal goal;
	bool costCalculated = false;
	float fCost = 0.0f, gCost = 0.0f, hCost = 0.0f;

	void calculateCosts();
	void calculateNeighbourCache();
};

class CBPathfinder
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
		CTConflict foundConflict{};
		std::map<size_t, std::vector<CATEntry>> constraints{};
		std::map<size_t, std::shared_ptr<pf::Path<PFData>>> solution{};
		float cost = 0.0f;
	};

	CBPathfinder(const std::vector<std::vector<bool>>& blockedGrid, const std::vector<ItemEndpoint>& itemEndpoints);
	float getFitness();
	size_t getPathsFound() const;
	void print();

private:
	const std::vector<std::vector<bool>>& blockedGrid;
	const std::vector<ItemEndpoint>& itemEndpoints;

	size_t currentPathGroup = 0;
	std::map<size_t, float> pathGroupSpareRates;
	std::vector<PathConfig> pathConfigs;
	std::shared_ptr<CTNode> finalSolution;
	bool finalSolutionFound = false;
	bool fitnessCalculated = false;
	float fitness = 0.0f;

	void solve();
	void preprocessPaths();
	void performPathfinding();
	void calculateNode(std::shared_ptr<CTNode> node);
	PFState* resolvePathConfig(size_t pathIndex, const std::shared_ptr<CTNode>& node, const ConflictAvoidanceTable& cat);
	std::tuple<bool, Coordinate, Direction> resolveBestPathEdge(const std::shared_ptr<pf::Path<PFData>>& path, const Coordinate& target, const std::shared_ptr<CTNode>& node, const ConflictAvoidanceTable& cat, const std::vector<CATEntry>& constraints);
};
