#define LOG_LOW_ENABLED 0
#define LOG_SOLVER_ENABLED 1
#define LOG_LOCAL_SEARCH_ENABLED 1

#ifndef LOG
#define LOG(level, ...)  \
	if (LOG_##level##_ENABLED) { \
		std::cout << __VA_ARGS__; \
	}
#endif

#define SRAND_SEED 9
#define USE_PATHFINDING 1
#define USE_ANNEALING 1
#define ANNEALING_TEMP 2.0f
#define ANNEALING_COOLING 0.005f
#define ANNEALING_ITERATIONS 1000
#define HILLCLIMBING_ITERATIONS 1000
#define USE_DATA_LOGGER 1
#define DATA_LOGGER_PREFIX "logs/tmp/sa_"
#define NOMINMAX

#include <windows.h>
#include <iostream>
#include <tuple>
#include <map>
#include <vector>
#include <set>
#include <stack>
#include <string>
#include <cassert>
#include <unordered_map>
#include "DataLogger.h"
#include "LocalSearch.h"
#include "Pathfinding.h"

void checkPathfinding();
void solveExampleProblem1();
void solveExampleProblem2();
void solveExampleProblem3();

int main()
{
	srand(SRAND_SEED);
	solveExampleProblem2();
}

//               Problem Definition
// ================================================

enum class BeltType : uint8_t { None = 1, Inserter = 2, Conveyor = 4, UndergroundEntrance = 8, Underground = 16, UndergroundExit = 32 };
enum class Direction : uint8_t { N = 1, S = 2, W = 4, E = 8 };

struct Coordinate { int x = 0; int y = 0; };

Direction dirOpposite(Direction dir)
{
	switch (dir)
	{
	case Direction::N: return Direction::S;
	case Direction::S: return Direction::N;
	case Direction::W: return Direction::E;
	case Direction::E: return Direction::W;
	}
	return Direction::N;
}

Coordinate dirOffset(Direction dir)
{
	switch (dir)
	{
	case Direction::N: return { 0, -1 };
	case Direction::S: return { 0, 1 };
	case Direction::W: return { -1, 0 };
	case Direction::E: return { 1, 0 };
	}
	return { 0, 0 };
}

std::string dirString(Direction dir)
{
	switch (dir)
	{
	case Direction::N: return "N";
	case Direction::S: return "S";
	case Direction::W: return "W";
	case Direction::E: return "E";
	}
	return "N";
}

const std::vector<Direction> INSERTER_DIRECTIONS = {
	Direction::N, Direction::N, Direction::N,
	Direction::E, Direction::E, Direction::E,
	Direction::S, Direction::S, Direction::S,
	Direction::W, Direction::W, Direction::W
};

const std::vector<Coordinate> INSERTER_OFFSETS = {
	{ 0, -1 }, { 1, -1 }, { 2, -1 },
	{ 3, 0 }, { 3, 1 }, { 3, 2 },
	{ 2, 3 }, { 1, 3 }, { 0, 3 },
	{ -1, 2 }, { -1, 1 }, { -1, 0 }
};

struct Recipe
{
	struct Ingredient
	{
		int item = -1;
		int quantity = 0;
	};

	int quantity = 0;
	float rate = 0;
	std::vector<Ingredient> ingredients;
};

struct ProblemDefinition
{
	struct ItemInput
	{
		int item = -1;
		float rate = 0;
		Coordinate coordinate;
	};

	struct ItemOutput
	{
		int item = -1;
		Coordinate coordinate;
	};

	int blueprintWidth = -1;
	int blueprintHeight = -1;
	std::map<int, Recipe> recipes;
	std::map<int, ItemInput> itemInputs;
	ItemOutput itemOutput;
};

class ProblemDefinitionFactory
{
public:
	static std::unique_ptr<ProblemDefinitionFactory> create()
	{
		return std::make_unique<ProblemDefinitionFactory>();
	}

	ProblemDefinitionFactory* setRecipes(std::map<int, Recipe> recipes)
	{
		problemDefinition.recipes = recipes;
		return this;
	}

	ProblemDefinitionFactory* setSize(int blueprintWidth, int blueprintHeight)
	{
		problemDefinition.blueprintWidth = blueprintWidth;
		problemDefinition.blueprintHeight = blueprintHeight;
		return this;
	}

	ProblemDefinitionFactory* addInputItem(int inputItem, float inputRate, int x, int y)
	{
		problemDefinition.itemInputs[inputItem] = { inputItem, inputRate, { x, y } };
		return this;
	}

	ProblemDefinitionFactory* addOutputItem(int itemOutput, int x, int y)
	{
		problemDefinition.itemOutput = { itemOutput, { x, y } };
		return this;
	}

	ProblemDefinition finalise()
	{
		return std::move(problemDefinition);
	}

private:
	ProblemDefinition problemDefinition;
};

//           Conflict Based Pathfinding
// ================================================

class PFState;

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
		std::string typeStr = type == BeltType::None ? "None" :
			type == BeltType::Conveyor ? "Conveyor" :
			type == BeltType::UndergroundEntrance ? "UndergroundEntrance" :
			type == BeltType::Underground ? "Underground" :
			type == BeltType::UndergroundExit ? "UndergroundExit" :
			type == BeltType::Inserter ? "Inserter"
			: "Unknown";
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

CATEntry calculateCATEntry(Coordinate coordinate, BeltType type, Direction direction)
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

bool checkCATEntryConflict(const CATEntry& a, const CATEntry& b)
{
	bool coordMatch = a.coordinateHash == b.coordinateHash;
	bool setOverlap = (a.conflictFlags & b.conflictFlags) != 0;
	return coordMatch && setOverlap;
}

class ConflictAvoidanceTable
{
public:
	ConflictAvoidanceTable() {}

	ConflictAvoidanceTable(const ConflictAvoidanceTable& other)
	{
		std::cout << "Start copy" << std::endl;
		for (const auto& coordinateEntry : other.table)
		{
			table[coordinateEntry.first] = std::vector<std::set<size_t>>();
			for (const auto& conflictSet : coordinateEntry.second)
			{
				std::set<size_t> newConflictSet = conflictSet;
				table[coordinateEntry.first].push_back(newConflictSet);
			}
		}
		std::cout << "End copy" << std::endl;
	}

	void addPath(size_t pathIndex, const CATEntry& entry)
	{
		if (table.find(entry.coordinateHash) == table.end())
		{
			table[entry.coordinateHash] = { {}, {}, {} };
		}

		if (entry.conflictFlags & 0b001) table[entry.coordinateHash][0].insert(pathIndex);
		if (entry.conflictFlags & 0b010) table[entry.coordinateHash][1].insert(pathIndex);
		if (entry.conflictFlags & 0b100) table[entry.coordinateHash][2].insert(pathIndex);
	}

	void removePath(size_t pathIndex)
	{
		for (auto& entry : table)
		{
			for (auto& conflictSet : entry.second)
			{
				conflictSet.erase(pathIndex);
			}
		}
	}

	bool checkConflict(const CATEntry& entry) const
	{
		auto found = table.find(entry.coordinateHash);
		if (found == table.end()) return false;

		const auto& conflictSets = found->second;
		if (entry.conflictFlags & 0b001 && conflictSets[0].size() > 1) return true;
		if (entry.conflictFlags & 0b010 && conflictSets[1].size() > 1) return true;
		if (entry.conflictFlags & 0b100 && conflictSets[2].size() > 1) return true;
		return false;
	}

	std::pair<CATEntry, std::set<size_t>> getConflictingPaths() const
	{
		for (const auto& entry : table)
		{
			const auto& conflictSets = entry.second;
			if (conflictSets[0].size() > 1)
			{
				return { { entry.first, 0b001 }, conflictSets[0] };
			}
			if (conflictSets[1].size() > 1)
			{
				return { { entry.first, 0b010 }, conflictSets[1] };
			}
			if (conflictSets[2].size() > 1)
			{
				return { { entry.first, 0b100 }, conflictSets[2] };
			}
		}
		return { { 0, 0 }, {} };
	}

private:
	std::map<size_t, std::vector<std::set<size_t>>> table;
};

class PFState : public pf::State<PFState, PFData>
{
public:
	PFState(const std::vector<std::vector<bool>>& blockedGrid, const std::vector<CATEntry>& constraints, PFGoal goal, PFData data)
		: blockedGrid(blockedGrid), constraints(constraints), goal(goal), pf::State<PFState, PFData>(data, nullptr)
	{}

	PFState(PFData data, PFState* parent)
		: pf::State<PFState, PFData>(data, parent), blockedGrid(parent->blockedGrid), constraints(parent->constraints), goal(parent->goal)
	{}

	bool isGoal() override
	{
		bool match = true;
		match &= data.coordinate.x == goal.coordinate.x && data.coordinate.y == goal.coordinate.y;
		if (goal.typeFlags > 0) match &= (goal.typeFlags & static_cast<uint8_t>(data.type)) > 0;
		if (goal.directionFlags > 0) match &= (goal.directionFlags & static_cast<uint8_t>(data.direction)) > 0;
		return match;
	}

	float getFCost() override
	{
		calculateCosts();
		return fCost;
	}

	float getGCost() override
	{
		calculateCosts();
		return gCost;
	}

	float getHCost() override
	{
		calculateCosts();
		return hCost;
	}

	std::vector<PFData*> getNeighbours() override
	{
		calculateNeighbourCache();

		// Calculate viable neighbours from cache that arent blocked or conflict
		std::vector<PFData*> neighbours;
		neighbours.reserve(neighbourCache[dataHash].size());

		for (size_t i = 0; i < neighbourCache[dataHash].size(); i++)
		{
			PFData& neighbourData = neighbourCache[dataHash][i].first;
			if (blockedGrid[neighbourData.coordinate.x][neighbourData.coordinate.y] && (neighbourData.type != BeltType::None && neighbourData.type != BeltType::Underground)) continue;

			const CATEntry& neighbourEntry = neighbourCache[dataHash][i].second;
			if (std::find_if(constraints.begin(), constraints.end(), [&](const auto& constraint) { return checkCATEntryConflict(constraint, neighbourEntry); }) != constraints.end()) continue;

			neighbours.push_back(&neighbourData);
		}
		return neighbours;
	}

	const PFGoal& getGoal() const
	{
		return goal;
	}

	bool conflictsWithConstraints() const
	{
		// Check if any entry in constraints conflicts with this state
		for (const CATEntry& entry : constraints)
		{
			if (checkCATEntryConflict(entry, calculateCATEntry(data.coordinate, data.type, data.direction))) return true;
		}
		return false;
	}

private:

	static std::unordered_map<size_t, std::vector<std::pair<PFData, CATEntry>>> neighbourCache;

	const std::vector<std::vector<bool>>& blockedGrid;
	const std::vector<CATEntry>& constraints;
	PFGoal goal;
	bool costCalculated = false;
	float fCost = 0.0f, gCost = 0.0f, hCost = 0.0f;

	void calculateCosts()
	{
		if (costCalculated) return;

		if (parent == nullptr) gCost = 0.0f;
		else
		{
			float coordDist = pf::ManhattanDistance((float)data.coordinate.x, (float)data.coordinate.y, (float)parent->data.coordinate.x, (float)parent->data.coordinate.y);
			if ((data.type == BeltType::Underground) || (parent->data.type == BeltType::Underground)) coordDist *= 0.5f;
			gCost = parent->gCost + coordDist;
		}

		hCost = pf::ManhattanDistance((float)data.coordinate.x, (float)data.coordinate.y, (float)goal.coordinate.x, (float)goal.coordinate.y);

		fCost = gCost + hCost;

		costCalculated = true;
	}

	void calculateNeighbourCache()
	{
		if (neighbourCache.find(dataHash) == neighbourCache.end())
		{
			neighbourCache[dataHash] = std::vector<std::pair<PFData, CATEntry>>();

			if (data.type == BeltType::None)
			{
				for (int i = 0; i < 4; i++)
				{
					Direction newDirection = static_cast<Direction>(1 << i);
					neighbourCache[dataHash].push_back({ { data.coordinate, BeltType::Conveyor, newDirection }, {} });
					neighbourCache[dataHash].push_back({ { data.coordinate, BeltType::UndergroundEntrance, newDirection }, {} });
				}
			}

			else if (data.type == BeltType::Conveyor)
			{
				for (int i = 0; i < 4; i++)
				{
					Direction newDirection = static_cast<Direction>(1 << i);
					Coordinate offset = dirOffset(newDirection);
					Coordinate newCoord = { data.coordinate.x + offset.x, data.coordinate.y + offset.y };
					if (newCoord.x < 0 || newCoord.x >= blockedGrid.size() || newCoord.y < 0 || newCoord.y >= blockedGrid[0].size()) continue;
					neighbourCache[dataHash].push_back({ { newCoord, BeltType::Conveyor, newDirection }, {} });
					neighbourCache[dataHash].push_back({ { newCoord, BeltType::UndergroundEntrance, newDirection }, {} });
				}
			}

			else if (data.type == BeltType::UndergroundEntrance)
			{
				Coordinate offset = dirOffset(data.direction);
				Coordinate newCoord = { data.coordinate.x + offset.x, data.coordinate.y + offset.y };
				if (newCoord.x >= 0 && newCoord.x < blockedGrid.size() && newCoord.y >= 0 && newCoord.y < blockedGrid[0].size())
				{
					neighbourCache[dataHash].push_back({ { newCoord, BeltType::Underground, data.direction }, {} });
					neighbourCache[dataHash].push_back({ { newCoord, BeltType::UndergroundExit, data.direction }, {} });
				}
			}

			else if (data.type == BeltType::Underground)
			{
				Coordinate offset = dirOffset(data.direction);
				Coordinate newCoord = { data.coordinate.x + offset.x, data.coordinate.y + offset.y };
				if (newCoord.x >= 0 && newCoord.x < blockedGrid.size() && newCoord.y >= 0 && newCoord.y < blockedGrid[0].size())
				{
					neighbourCache[dataHash].push_back({ { newCoord, BeltType::Underground, data.direction }, {} });
					neighbourCache[dataHash].push_back({ { newCoord, BeltType::UndergroundExit, data.direction }, {} });
				}
			}

			else if (data.type == BeltType::UndergroundExit)
			{
				Coordinate offset = dirOffset(data.direction);
				Coordinate newCoord = { data.coordinate.x + offset.x, data.coordinate.y + offset.y };
				if (newCoord.x >= 0 && newCoord.x < blockedGrid.size() && newCoord.y >= 0 && newCoord.y < blockedGrid[0].size())
				{
					neighbourCache[dataHash].push_back({ { newCoord, BeltType::Conveyor, data.direction }, {} });
					neighbourCache[dataHash].push_back({ { newCoord, BeltType::UndergroundEntrance, data.direction }, {} });
				}
			}

			else if (data.type == BeltType::Inserter)
			{
				Coordinate offset = dirOffset(data.direction);
				Coordinate newCoord = { data.coordinate.x + offset.x, data.coordinate.y + offset.y };
				if (newCoord.x >= 0 && newCoord.x < blockedGrid.size() && newCoord.y >= 0 && newCoord.y < blockedGrid[0].size())
				{
					for (int i = 0; i < 4; i++)
					{
						Direction newDirection = Direction(1 << i);
						if (data.direction == dirOpposite(newDirection)) continue;
						neighbourCache[dataHash].push_back({ { newCoord, BeltType::Conveyor, newDirection }, {} });
						neighbourCache[dataHash].push_back({ { newCoord, BeltType::UndergroundEntrance, newDirection }, {} });
					}
				}
			}

			for (auto& neighbour : neighbourCache[dataHash])
			{
				neighbour.second = calculateCATEntry(neighbour.first.coordinate, neighbour.first.type, neighbour.first.direction);
			}
		}
	}
};

struct RunConfig
{
	struct ItemConfig
	{
		struct InserterRequirement { int count; float rate; };

		int item = -1;
		int assemblerCount = 0;
		InserterRequirement outputInserterRequirement;
		std::map<int, InserterRequirement> inputInserterRequirements;
	};

	int outputAssemblerCount = 0;
	std::map<int, ItemConfig> itemConfigs;
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

class CBPathfinder
{
public:
	static size_t evaluationCount;

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

	CBPathfinder(const std::vector<std::vector<bool>>& blockedGrid, const std::vector<ItemEndpoint>& itemEndpoints)
		: blockedGrid(blockedGrid), itemEndpoints(itemEndpoints)
	{}

	float getFitness()
	{
		if (fitnessCalculated) return fitness;
		CBPathfinder::evaluationCount++;

		solve();

		fitnessCalculated = true;
		return fitness;
	}

	size_t getPathsFound() const
	{
		if (!finalSolutionFound) return 0;
		size_t pathsFound = 0;
		for (const auto& path : finalSolution->solution)
		{
			if (path.second->found) pathsFound++;
		}
		return pathsFound;
	}

	void print()
	{
		std::cout << "--- CB Pathfinding ---" << std::endl << std::endl;

		if (!finalSolutionFound)
		{
			std::cout << "No solution found" << std::endl;
			return;
		}

		std::cout << "Solution found, fitness: " << fitness << std::endl << std::endl;

		std::cout << "Constraints: ( ";
		for (const auto& entry : finalSolution->constraints)
		{
			std::cout << entry.second.size() << " ";
		}
		std::cout << ")" << std::endl;

		auto printEndpoint = [&](const ConcreteEndpoint& endpoint)
		{
			if (endpoint.type == ConcreteEndpoint::Type::ITEM)
			{
				const ItemEndpoint& itemEndpoint = itemEndpoints[endpoint.index];
				std::cout << "[ Item " << itemEndpoint.item << " at (" << itemEndpoint.coordinate.x << ", " << itemEndpoint.coordinate.y << ") ]";
			}
			else
			{
				const PathConfig& pathConfig = pathConfigs[endpoint.index];
				std::cout << "[ Path " << endpoint.index << " ]";
			}
		};

		for (size_t i = 0; i < finalSolution->solution.size(); i++)
		{
			const auto& path = finalSolution->solution[i];
			std::cout << "Path " << i << " ";
			printEndpoint(pathConfigs[i].source);
			std::cout << " to ";
			printEndpoint(pathConfigs[i].destination);
			std::cout << ", cost: " << path->cost;
			std::cout << ", nodes: " << path->nodes.size() << std::endl;

			for (const auto& nodeData : path->nodes)
			{
				std::cout << "- ";
				nodeData.print();
			}
		}

		std::cout << std::endl;
	}

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

	void solve()
	{
		fitness = 0.0f;
		preprocessPaths();
		performPathfinding();
	}

	void preprocessPaths()
	{
		pathConfigs = std::vector<PathConfig>();

		// LOGGING: Path endpoints
		LOG(LOW, "--- CB Item Endpoints ---\n");
		for (size_t i = 0; i < this->itemEndpoints.size(); i++)
		{
			LOG(LOW, "Endpoint " << i << ": item " << this->itemEndpoints[i].item
				<< " at (" << this->itemEndpoints[i].coordinate.x << ", " << this->itemEndpoints[i].coordinate.y << ") "
				<< (this->itemEndpoints[i].isSource ? "source" : "destination")
				<< " @ " << this->itemEndpoints[i].rate << "/s\n");
		}
		LOG(LOW, "\n--- CB Endpoint Processing ---\n");

		// Seperate end points by item
		std::map<int, std::vector<size_t>> itemsEndpoints = std::map<int, std::vector<size_t>>();
		for (size_t i = 0; i < this->itemEndpoints.size(); i++)
		{
			const ItemEndpoint& endpoint = this->itemEndpoints[i];
			if (itemsEndpoints.find(endpoint.item) == itemsEndpoints.end())
			{
				itemsEndpoints[endpoint.item] = std::vector<size_t>();
			}
			itemsEndpoints[endpoint.item].push_back(i);
		}

		// Process each items endpoints
		for (auto& item : itemsEndpoints)
		{
			// Sort endpoints based on rate
			std::sort(item.second.begin(), item.second.end(), [&](size_t a, size_t b) { return this->itemEndpoints[a].rate < this->itemEndpoints[b].rate; });
			auto& endpoints = item.second;

			// Keep track of paths for this item
			std::set<size_t> pathConfigs = std::set<size_t>();

			// While there are endpoints to process, grab the highest rate
			while (endpoints.size() > 0)
			{
				size_t currentIndex = endpoints.back();
				endpoints.pop_back();
				const ItemEndpoint& current = this->itemEndpoints[currentIndex];

				// Find the most suitable endpoint, minimising |spareRate|, and prioritising > 0
				float bestSpareRate = std::numeric_limits<float>::max();
				ConcreteEndpoint bestEndpoint{};
				bool bestIsPriority = false;

				// Check all compatible item endpoints
				for (size_t i = 0; i < endpoints.size(); i++)
				{
					size_t otherIndex = endpoints[i];
					const ItemEndpoint& other = this->itemEndpoints[otherIndex];
					if (current.isSource == other.isSource) continue;

					float spareRate;
					if (current.isSource) spareRate = current.rate - other.rate;
					else spareRate = other.rate - current.rate;
					bool isPriority = spareRate >= 0;

					// Consider priority, then smallest absolute spare rate
					if ((!bestIsPriority && isPriority) || ((bestIsPriority == isPriority) && (abs(spareRate) < abs(bestSpareRate))))
					{
						bestSpareRate = spareRate;
						bestEndpoint = { ConcreteEndpoint::Type::ITEM, otherIndex };
						bestIsPriority = isPriority;
					}
				}

				// Check all paths for this item
				for (size_t pathIndex : pathConfigs)
				{
					const PathConfig& path = this->pathConfigs[pathIndex];

					float currentSpareRate = this->pathGroupSpareRates[path.pathGroup];
					float newSpareRate;
					if (current.isSource) newSpareRate = current.rate + currentSpareRate;
					else newSpareRate = currentSpareRate - current.rate;
					bool isPriority = newSpareRate >= 0;

					// Consider priority, then smallest absolute spare rate
					if ((!bestIsPriority && isPriority) || (bestIsPriority == isPriority && (abs(newSpareRate) < abs(bestSpareRate))))
					{
						bestSpareRate = newSpareRate;
						bestEndpoint = { ConcreteEndpoint::Type::PATH, pathIndex };
						bestIsPriority = isPriority;
					}
				}

				// Create the concrete endpoint for the current item endpoint
				ConcreteEndpoint currentEndpoint = { ConcreteEndpoint::Type::ITEM, currentIndex };

				// Add a path with the current and best endpoint
				size_t pathIndex = this->pathConfigs.size();
				size_t pathGroup = (bestEndpoint.type == ConcreteEndpoint::Type::PATH) ? this->pathConfigs[bestEndpoint.index].pathGroup : this->currentPathGroup++;
				if (current.isSource) this->pathConfigs.push_back({ currentEndpoint, bestEndpoint, pathGroup });
				else this->pathConfigs.push_back({ bestEndpoint, currentEndpoint, pathGroup });
				pathConfigs.insert(pathIndex);

				// Add the path to the dependant paths of the best endpoint
				if (bestEndpoint.type == ConcreteEndpoint::Type::PATH)
				{
					this->pathConfigs[bestEndpoint.index].dependantPaths.push_back(pathIndex);
				}

				// Remove the best endpoint from the list of endpoints
				else if (bestEndpoint.type == ConcreteEndpoint::Type::ITEM)
				{
					endpoints.erase(std::remove(endpoints.begin(), endpoints.end(), bestEndpoint.index), endpoints.end());
				}

				// LOGGING: Picked choices and path
				LOG(LOW, "Current: index " << currentIndex << ", item " << current.item << " at (" << current.coordinate.x << ", " << current.coordinate.y << ") "
					<< (current.isSource ? "source" : "destination") << " @ " << current.rate << "/s\n");
				if (bestEndpoint.type == ConcreteEndpoint::Type::ITEM)
				{
					const ItemEndpoint& other = this->itemEndpoints[bestEndpoint.index];
					LOG(LOW, "Other (world): index " << bestEndpoint.index << " at (" << other.coordinate.x << ", " << other.coordinate.y << ") "
						<< (other.isSource ? "source" : "destination") << " @ " << other.rate << "/s\n");
				}
				else
				{
					const PathConfig& path = this->pathConfigs[bestEndpoint.index];
					float groupSpareRate = this->pathGroupSpareRates[path.pathGroup];
					LOG(LOW, "Other (path): index " << bestEndpoint.index << " spare rate @ " << groupSpareRate << "/s\n");
				}
				LOG(LOW, "Created path: index " << pathIndex << " group " << pathGroup << " new spare rate @ " << bestSpareRate << "/s\n\n");

				// Update the spare rate of the path group
				this->pathGroupSpareRates[pathGroup] = bestSpareRate;
			}
		}

		// LOGGING: Print out paths
		LOG(LOW, "--- Final Path configs ---\n");
		for (size_t i = 0; i < this->pathConfigs.size(); i++)
		{
			const PathConfig& path = this->pathConfigs[i];
			LOG(LOW, "Path " << i << ": group " << path.pathGroup << " from [");
			if (path.source.type == ConcreteEndpoint::Type::ITEM)
			{
				const ItemEndpoint& source = this->itemEndpoints[path.source.index];
				LOG(LOW, "index " << path.source.index << " item " << source.item
					<< " at (" << source.coordinate.x << ", " << source.coordinate.y << ") "
					<< (source.isSource ? "source" : "destination")
					<< " @ " << source.rate << "/s");
			}
			else
			{
				const PathConfig& source = this->pathConfigs[path.source.index];
				LOG(LOW, "index " << path.source.index << " group " << source.pathGroup << " @ " << this->pathGroupSpareRates[source.pathGroup] << "/s");
			}
			LOG(LOW, "] to [");
			if (path.destination.type == ConcreteEndpoint::Type::ITEM)
			{
				const ItemEndpoint& destination = this->itemEndpoints[path.destination.index];
				LOG(LOW, "index " << path.destination.index << " item " << destination.item
					<< " at (" << destination.coordinate.x << ", " << destination.coordinate.y << ") "
					<< (destination.isSource ? "source" : "destination")
					<< " @ " << destination.rate << "/s");
			}
			else
			{
				const PathConfig& destination = this->pathConfigs[path.destination.index];
				LOG(LOW, "index " << path.destination.index << " group " << destination.pathGroup << " @ " << this->pathGroupSpareRates[destination.pathGroup] << "/s");
			}
			LOG(LOW, "]\n");
		}
		LOG(LOW, "\n");
	}

	void performPathfinding()
	{
		LOG(LOW, "--- CB Pathfinding ---\n\n");

		// NOTES
		//
		// A CTNode will be invalid if a path config cannot be resolved
		// The openSet will eventually be empty if all the CTNodes end up invalid
		// It is possible for a solution with no conflicts, but some paths not found
		//
		// Recreating the CAT each time in calculateNode is tradeoff between memory and CPU
		// - Copying the CAT over could actually be slower eitherway
		// Potentially need to look at the tradeoff between holding all constraints and copying
		// - These are very small structs so likely not an issue

		// Initialize root and early exit if invalid
		auto root = std::make_shared<CTNode>();
		root->isValid = true;
		for (size_t i = 0; i < this->pathConfigs.size(); i++) root->pathsToCalculate.push_back(i);
		calculateNode(root);
		if (!root->isValid) return;

		// Initialize open set
		std::vector<std::shared_ptr<CTNode>> openSet;
		openSet.push_back(root);

		// While there are nodes to process
		this->finalSolutionFound = false;
		while (openSet.size() > 0)
		{
			// Pop the node with the lowest cost
			auto current = openSet[0];
			for (const auto& node : openSet)
			{
				if (node->cost < current->cost) current = node;
			}
			openSet.erase(std::remove(openSet.begin(), openSet.end(), current), openSet.end());

			// TODO: Matching conflicts seem to just be infinitely going up
			// LOGGING: Print out current node
			LOG(LOW, "Open set: " << openSet.size() << ", cost: " << current->cost << ", Constraints: ( ");
			for (const auto& entry : current->constraints)
			{
				LOG(LOW, entry.second.size() << " ");
			}
			LOG(LOW, ")\n");

			// If no conflict, have found a solution
			if (!current->foundConflict.isConflict)
			{
				this->finalSolutionFound = true;
				this->finalSolution = current;
				break;
			}

			// Create new nodes for each conflicting path
			for (size_t pathIndex : { current->foundConflict.pathA, current->foundConflict.pathB })
			{
				// Initialize next node as a copy
				auto next = std::make_shared<CTNode>();
				next->isValid = true;
				next->constraints = current->constraints;
				next->solution = current->solution;
				next->cost = current->cost;

				// Set the path to calculate and add the constraint
				next->pathsToCalculate = { pathIndex };
				next->constraints[pathIndex].push_back(current->foundConflict.catEntry);

				// Calculate the node and add to open set if valid
				calculateNode(next);
				if (next->isValid) openSet.push_back(next);
			}
		}

		// No solution without conflicts found, fitness = 0
		if (!this->finalSolutionFound)
		{
			LOG(LOW, "\nNo solution found\n\n");
			fitness = 0.0f;
			return;
		}

		// Solution found, fitness = sum (1 + shortest / real)
		LOG(LOW, "\nSolution found\n\n");

		fitness = 0.0f;
		for (size_t i = 0; i < this->pathConfigs.size(); i++)
		{
			if (!this->finalSolution->solution[i]->found)
			{
				LOG(LOW, "Path " << i << " in the solution, could not found\n");
				continue;
			}

			const auto& first = this->finalSolution->solution[i]->nodes[0];
			const auto& last = this->finalSolution->solution[i]->nodes.back();
			float shortest = pf::ManhattanDistance((float)first.coordinate.x, (float)first.coordinate.y, (float)last.coordinate.x, (float)last.coordinate.y);
			float real = this->finalSolution->solution[i]->cost;

			// Its possible with underground belts to have a lower cost, but cap it to max
			real = std::max(real, shortest);

			if (shortest == 0) fitness += 2.0f;
			else fitness += (1.0f + shortest / real);

			LOG(LOW, "Path " << i << " in the solution, shortest " << shortest << ", real " << real << ", fitness " << (1.0f + shortest / real) << "\n");
		}

		LOG(LOW, "\n");
	}

	void calculateNode(std::shared_ptr<CTNode> node)
	{
		// Exit early if no paths to calculate
		if (node->pathsToCalculate.size() == 0) return;
		node->isValid = false;
		node->cost = 0.0f;

		// Ensure all dependant paths are calculated in the right order
		std::vector<size_t> calculateOrder{};
		while (node->pathsToCalculate.size() > 0)
		{
			size_t pathIndex = node->pathsToCalculate.back();
			node->pathsToCalculate.pop_back();

			// If already in order, move to end
			auto found = std::find(calculateOrder.begin(), calculateOrder.end(), pathIndex);
			if (found != calculateOrder.end()) calculateOrder.erase(found);
			calculateOrder.push_back(pathIndex);

			// Add all dependant paths in the queue
			for (size_t dependantPathIndex : pathConfigs[pathIndex].dependantPaths)
			{
				node->pathsToCalculate.push_back(dependantPathIndex);
			}
		}

		// Initialize CAT with non-calculated paths and update cost
		ConflictAvoidanceTable cat;
		for (size_t i = 0; i < this->pathConfigs.size(); i++)
		{
			if (std::find(calculateOrder.begin(), calculateOrder.end(), i) == calculateOrder.end())
			{
				node->cost += node->solution[i]->cost;
				for (const auto& nodeData : node->solution[i]->nodes) cat.addPath(i, calculateCATEntry(nodeData.coordinate, nodeData.type, nodeData.direction));
			}
		}

		// Calculate each path in order and update CAT and cost
		for (size_t pathIndex : calculateOrder)
		{
			auto source = resolvePathConfig(pathIndex, node, cat);
			if (source == nullptr) return;
			node->solution[pathIndex] = pf::asPathfinding<PFState, PFData>(source, true);
			node->cost += node->solution[pathIndex]->cost;
			for (const auto& nodeData : node->solution[pathIndex]->nodes) cat.addPath(pathIndex, calculateCATEntry(nodeData.coordinate, nodeData.type, nodeData.direction));
		}

		// Find conflict for node and update
		auto conflict = cat.getConflictingPaths();
		if (conflict.second.size() == 0)
		{
			node->foundConflict = { false };
		}
		else
		{
			auto it = conflict.second.begin();
			size_t pathA = *it;
			size_t pathB = *(++it);
			node->foundConflict = { true, pathA, pathB, conflict.first };
		}

		// Node is valid once all paths found
		node->isValid = true;
	}

	PFState* resolvePathConfig(size_t pathIndex, const std::shared_ptr<CTNode>& node, const ConflictAvoidanceTable& cat)
	{
		// Attempt to resolve the path considering the given node
		// Return nullptr is conflicts and constraints prevent resolution
		const PathConfig& path = this->pathConfigs[pathIndex];
		const auto& constraints = node->constraints[pathIndex];

		// Position of destination -> goal, resolve closest source path edge -> PFState
		if (path.source.type == ConcreteEndpoint::Type::PATH)
		{
			const ItemEndpoint& destinationEndpoint = this->itemEndpoints[path.destination.index];

			const auto destinationCATEntry = calculateCATEntry(destinationEndpoint.coordinate, BeltType::Conveyor, Direction::N);
			if (std::any_of(constraints.begin(), constraints.end(), [&](const CATEntry& entry) { return checkCATEntryConflict(entry, destinationCATEntry); })) return nullptr;
			if (cat.checkConflict(destinationCATEntry)) return nullptr;

			const auto& sourcePath = node->solution[path.source.index];
			auto best = resolveBestPathEdge(sourcePath, destinationEndpoint.coordinate, node, cat, constraints);
			if (std::get<0>(best) == false) return nullptr;

			PFGoal goal{
				destinationEndpoint.coordinate,
				static_cast<uint8_t>(BeltType::Conveyor) |
				static_cast<uint8_t>(BeltType::UndergroundEntrance) |
				static_cast<uint8_t>(BeltType::UndergroundExit),
				0
			};

			return new PFState(blockedGrid, node->constraints[pathIndex], goal, PFData{ std::get<1>(best), BeltType::Inserter, std::get<2>(best) });
		}

		// Position of source -> PFState, resolve closest destination path edge -> goal
		if (path.destination.type == ConcreteEndpoint::Type::PATH)
		{
			const ItemEndpoint& sourceEndpoint = this->itemEndpoints[path.source.index];

			const auto sourceCATEntry = calculateCATEntry(sourceEndpoint.coordinate, BeltType::Conveyor, Direction::N);
			if (std::any_of(constraints.begin(), constraints.end(), [&](const CATEntry& entry) { return checkCATEntryConflict(entry, sourceCATEntry); })) return nullptr;
			if (cat.checkConflict(sourceCATEntry)) return nullptr;

			const auto& destinationPath = node->solution[path.destination.index];
			auto best = resolveBestPathEdge(destinationPath, sourceEndpoint.coordinate, node, cat, constraints);
			if (std::get<0>(best) == false) return nullptr;

			PFGoal goal{
				std::get<1>(best),
				static_cast<uint8_t>(BeltType::Conveyor) |
				static_cast<uint8_t>(BeltType::UndergroundExit),
				static_cast<uint8_t>(dirOpposite(std::get<2>(best)))
			};

			return new PFState(blockedGrid, node->constraints[pathIndex], goal, PFData{ sourceEndpoint.coordinate, BeltType::None, Direction::N });
		}

		// Position of source -> PFState, position of destination -> goal
		const ItemEndpoint& sourceEndpoint = this->itemEndpoints[path.source.index];
		const ItemEndpoint& destinationEndpoint = this->itemEndpoints[path.destination.index];

		const auto sourceCATEntry = calculateCATEntry(sourceEndpoint.coordinate, BeltType::Conveyor, Direction::N);
		const auto destinationCATEntry = calculateCATEntry(destinationEndpoint.coordinate, BeltType::Conveyor, Direction::N);
		if (std::any_of(constraints.begin(), constraints.end(), [&](const CATEntry& entry) { return checkCATEntryConflict(entry, sourceCATEntry) || checkCATEntryConflict(entry, destinationCATEntry); })) return nullptr;
		if (cat.checkConflict(sourceCATEntry) || cat.checkConflict(destinationCATEntry)) return nullptr;

		PFGoal goal{
			destinationEndpoint.coordinate,
			static_cast<uint8_t>(BeltType::Conveyor) |
			static_cast<uint8_t>(BeltType::UndergroundEntrance) |
			static_cast<uint8_t>(BeltType::UndergroundExit),
			0
		};
		return new PFState(blockedGrid, node->constraints[pathIndex], goal, PFData{ sourceEndpoint.coordinate, BeltType::None, Direction::N });
	}

	std::tuple<bool, Coordinate, Direction> resolveBestPathEdge(const std::shared_ptr<pf::Path<PFData>>& path, const Coordinate& target, const std::shared_ptr<CTNode>& node, const ConflictAvoidanceTable& cat, const std::vector<CATEntry>& constraints)
	{
		// Look for the closest edge of the path to the target
		float bestDistance = std::numeric_limits<float>::max();
		Coordinate bestCoord = { -1, -1 };
		Direction bestDir = Direction::N;
		bool found = false;

		// For each above ground node in the path, check the 4 directions
		for (size_t i = 0; i < path->nodes.size(); i++)
		{
			const auto& nodeData = path->nodes[i];
			if (nodeData.isAboveGround())
			{
				const Coordinate& coord = nodeData.coordinate;
				for (int j = 0; j < 4; j++)
				{
					Direction dir = static_cast<Direction>(1 << j);
					Coordinate offset = dirOffset(dir);

					Coordinate newCoord = { coord.x + offset.x, coord.y + offset.y };
					CATEntry catEntry = calculateCATEntry(newCoord, BeltType::Conveyor, dir);

					// Only if not blocked, constrained, or have conflicting CAT entries
					if (newCoord.x < 0 || newCoord.x >= blockedGrid.size() || newCoord.y < 0 || newCoord.y >= blockedGrid[0].size()) continue;
					if (blockedGrid[newCoord.x][newCoord.y]) continue;
					if (cat.checkConflict(catEntry)) continue;
					if (std::any_of(constraints.begin(), constraints.end(), [&](const CATEntry& entry) { return checkCATEntryConflict(entry, catEntry); })) continue;

					// Check the extra space is also valid
					Coordinate extraCoord = { newCoord.x + offset.x, newCoord.y + offset.y };
					CATEntry extraCatEntry = calculateCATEntry(extraCoord, BeltType::Conveyor, dir);

					if (extraCoord.x < 0 || extraCoord.x >= blockedGrid.size() || extraCoord.y < 0 || extraCoord.y >= blockedGrid[0].size()) continue;
					if (blockedGrid[extraCoord.x][extraCoord.y]) continue;
					if (cat.checkConflict(extraCatEntry)) continue;
					if (std::any_of(constraints.begin(), constraints.end(), [&](const CATEntry& entry) { return checkCATEntryConflict(entry, extraCatEntry); })) continue;

					// Check distance to target
					float distance = pf::ManhattanDistance((float)newCoord.x, (float)newCoord.y, (float)target.x, (float)target.y);
					if (distance < bestDistance)
					{
						bestDistance = distance;
						bestCoord = newCoord;
						bestDir = dir;
						found = true;
					}
				}
			}
		}

		return { found, bestCoord, bestDir };
	}
};

//            Placement Local Search
// ================================================

class LSState : public ls::State<LSState>
{
public:
	static size_t evaluationCount;

	struct InserterInstance
	{
		int item = -1;
		float rate = 0.0f;
		bool isInput = false;
	};

	struct AssemblerInstance
	{
		static const std::vector<Direction> inserterDirections;
		static const std::vector<Coordinate> inserterOffsets;

		int item = -1;
		Coordinate coordinate;
		InserterInstance inserters[12];
	};

public:
	static std::shared_ptr<LSState> createRandom(const ProblemDefinition& problem, const RunConfig& runConfig)
	{
		std::vector<AssemblerInstance> assemblers;

		// For each item type
		for (const auto& itemConfigPair : runConfig.itemConfigs)
		{
			const auto& itemConfig = itemConfigPair.second;
			if (itemConfig.assemblerCount == 0) continue;

			// Randomly place assemblers
			for (int i = 0; i < itemConfig.assemblerCount; i++)
			{
				Coordinate coord;
				coord.x = rand() % (problem.blueprintWidth - 2);
				coord.y = rand() % (problem.blueprintHeight - 2);
				AssemblerInstance assembler{ itemConfig.item, coord };

				// Available inserters for this assembler
				std::vector<int> availableInserters(12);
				for (int i = 0; i < 12; i++) availableInserters[i] = i;

				// Randomly place input inserters
				for (const auto& inputRequirement : itemConfig.inputInserterRequirements)
				{
					for (int i = 0; i < inputRequirement.second.count; i++)
					{
						int index = rand() % availableInserters.size();
						int inserterIndex = availableInserters[index];
						availableInserters.erase(availableInserters.begin() + index);
						assembler.inserters[inserterIndex] = InserterInstance{ inputRequirement.first, inputRequirement.second.rate, true };
					}
				}

				// Randomly place output inserters
				for (int i = 0; i < itemConfig.outputInserterRequirement.count; i++)
				{
					int index = rand() % availableInserters.size();
					int inserterIndex = availableInserters[index];
					availableInserters.erase(availableInserters.begin() + index);
					assembler.inserters[inserterIndex] = InserterInstance{ itemConfig.item, itemConfig.outputInserterRequirement.rate, false };
				}

				assemblers.push_back(assembler);
			}
		}

		return std::make_shared<LSState>(problem, runConfig, assemblers);
	}

	LSState(const ProblemDefinition& problem, const RunConfig& runConfig, const std::vector<AssemblerInstance>& assemblers)
		: problem(problem), runConfig(runConfig), assemblers(assemblers)
	{
		calculateHash();
	}

	float getFitness() override
	{
		if (costCalculated) return fitness;
		LSState::evaluationCount++;

		fitness = 0.0f;

		calculateWorld();
		fitness -= worldCost;

		#if USE_PATHFINDING
		if (isWorldValid)
		{
			pathfinder = std::make_shared<CBPathfinder>(blockedGrid, itemEndpoints);
			fitness += pathfinder->getFitness();
		}
		#endif

		costCalculated = true;
		return fitness;
	}

	std::vector<std::shared_ptr<LSState>> getNeighbours() override
	{
		if (neighboursCalculated) return neighbours;

		neighbours = std::vector<std::shared_ptr<LSState>>();

		for (int i = 0; i < assemblers.size(); i++)
		{
			const AssemblerInstance& assembler = assemblers[i];

			// Neighbour for moving each assembler in each direction
			for (int j = 0; j < 4; j++)
			{
				Direction dir = static_cast<Direction>(1 << j);
				Coordinate offset = dirOffset(dir);
				Coordinate newCoord = { assembler.coordinate.x + offset.x, assembler.coordinate.y + offset.y };

				if (newCoord.x < 0 || newCoord.x >= problem.blueprintWidth - 3 || newCoord.y < 0 || newCoord.y >= problem.blueprintHeight - 3) continue;

				std::vector<AssemblerInstance> newAssemblers = assemblers;
				newAssemblers[i].coordinate = newCoord;
				neighbours.push_back(getCached(std::make_shared<LSState>(problem, runConfig, newAssemblers)));
			}

			// Neighbour for moving an inserter from 1 slot to another on the same assembler
			for (int j = 0; j < 12; j++)
			{
				const InserterInstance& inserter = assembler.inserters[j];
				if (inserter.item == -1) continue;

				for (int k = 0; k < 12; k++)
				{
					if (j == k) continue;
					const InserterInstance& otherInserter = assembler.inserters[k];
					if (otherInserter.item != -1) continue;

					std::vector<AssemblerInstance> newAssemblers = assemblers;
					newAssemblers[i].inserters[j] = InserterInstance{ -1, false };
					newAssemblers[i].inserters[k] = inserter;
					neighbours.push_back(getCached(std::make_shared<LSState>(problem, runConfig, newAssemblers)));
				}
			}
		}

		neighboursCalculated = true;
		return neighbours;
	}

	bool getValid()
	{
		calculateWorld();
		return isWorldValid;
	}

	void print()
	{
		calculateWorld();

		std::cout << "--- Local State World ---" << std::endl << std::endl;

		HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);

		for (int y = 0; y < problem.blueprintHeight; y++)
		{
			for (int x = 0; x < problem.blueprintWidth; x++)
			{
				if (blockedGrid[x][y])
				{
					if (blockedGrid[x][y]) SetConsoleTextAttribute(hConsole, 15 + 1 * 16);
					std::cout << blockedGrid[x][y];
				}
				else if (itemGrid[x][y] > -1)
				{
					SetConsoleTextAttribute(hConsole, 15 + 3 * 16);
					std::cout << itemGrid[x][y];
				}
				else
				{
					SetConsoleTextAttribute(hConsole, 15);
					std::cout << " ";
				}
			}
			SetConsoleTextAttribute(hConsole, 15);
			std::cout << " " << std::endl;
		}

		SetConsoleTextAttribute(hConsole, 15);
		std::cout << std::endl;

		std::cout << "--- Local State Data ---" << std::endl << std::endl;

		for (const auto& assembler : assemblers)
		{
			std::cout << "Assembler: item " << assembler.item
				<< " at (" << assembler.coordinate.x << ", " << assembler.coordinate.y << ")" << std::endl;

			for (int i = 0; i < 12; i++)
			{
				const InserterInstance& inserter = assembler.inserters[i];
				if (inserter.item != -1)
				{
					std::cout << "  Inserter: index " << i
						<< " item " << inserter.item
						<< (inserter.isInput ? " input" : " output")
						<< " at (" << assembler.coordinate.x + INSERTER_OFFSETS[i].x << ", " << assembler.coordinate.y + INSERTER_OFFSETS[i].y << ")"
						<< " @ " << inserter.rate << "/s" << std::endl;
				}
			}
		}

		std::cout << std::endl;

		if (pathfinder != nullptr)
		{
			pathfinder->print();
		}
	}

	std::vector<float> generateDataLog() override
	{
		std::vector<float>  log;

		// Log fitness
		log.push_back(fitness);

		// Log number of paths CBS found
		if (pathfinder == nullptr) log.push_back(0.0f);
		else log.push_back((float)pathfinder->getPathsFound());

		return log;
	}

private:
	const ProblemDefinition& problem;
	const RunConfig& runConfig;

	std::vector<AssemblerInstance> assemblers;

	bool costCalculated = false;
	bool neighboursCalculated = false;
	bool worldCalculated = false;
	bool isWorldValid = false;

	std::vector<std::shared_ptr<LSState>> neighbours;
	std::vector<std::vector<bool>> blockedGrid;
	std::vector<std::vector<int>> itemGrid;
	std::vector<ItemEndpoint> itemEndpoints;
	float worldCost = 0.0f;
	std::shared_ptr<CBPathfinder> pathfinder;
	float fitness = 0.0f;

	void calculateWorld()
	{
		if (worldCalculated) return;

		itemEndpoints = std::vector<ItemEndpoint>();
		blockedGrid = std::vector<std::vector<bool>>(problem.blueprintWidth, std::vector<bool>(problem.blueprintHeight, false));
		itemGrid = std::vector<std::vector<int>>(problem.blueprintWidth, std::vector<int>(problem.blueprintHeight, -1));

		// Add input and output items to world
		for (const auto& input : problem.itemInputs)
		{
			itemGrid[input.second.coordinate.x][input.second.coordinate.y] = input.first;
			itemEndpoints.push_back({ input.first, input.second.rate, true, input.second.coordinate });
		}
		itemGrid[problem.itemOutput.coordinate.x][problem.itemOutput.coordinate.y] = problem.itemOutput.item;
		itemEndpoints.push_back({ problem.itemOutput.item, 0.0f, false, problem.itemOutput.coordinate });

		for (const auto& assembler : assemblers)
		{
			// Check each 3x3 area in each assembler for blocked
			for (int x = 0; x < 3; x++)
			{
				for (int y = 0; y < 3; y++)
				{
					Coordinate coord = { assembler.coordinate.x + x, assembler.coordinate.y + y };

					if (blockedGrid[coord.x][coord.y]) worldCost += 1.0f;
					if (itemGrid[coord.x][coord.y] != -1) worldCost += 1.0f;

					blockedGrid[coord.x][coord.y] = true;
				}
			}

			// Check each of 12 inserter positions for blocked
			for (int i = 0; i < 12; i++)
			{
				const InserterInstance& inserter = assembler.inserters[i];
				if (inserter.item == -1) continue;

				Coordinate offset = INSERTER_OFFSETS[i];
				Coordinate coord = { assembler.coordinate.x + offset.x, assembler.coordinate.y + offset.y };

				if (coord.x < 0 || coord.x >= problem.blueprintWidth || coord.y < 0 || coord.y >= problem.blueprintHeight)
				{
					worldCost += 1.0f;
					continue;
				}

				if (blockedGrid[coord.x][coord.y]) worldCost += 1.0f;
				if (itemGrid[coord.x][coord.y] != -1) worldCost += 1.0f;

				blockedGrid[coord.x][coord.y] = true;

				// Check inserter open side for items or blocked
				Direction checkDir = INSERTER_DIRECTIONS[i];
				Coordinate checkOffset = dirOffset(checkDir);
				Coordinate checkCoord = { coord.x + checkOffset.x, coord.y + checkOffset.y };

				if (checkCoord.x < 0 || checkCoord.x >= problem.blueprintWidth || checkCoord.y < 0 || checkCoord.y >= problem.blueprintHeight)
				{
					worldCost += 1.0f;
					continue;
				}

				if (blockedGrid[checkCoord.x][checkCoord.y]) worldCost += 1.0f;
				if (itemGrid[checkCoord.x][checkCoord.y] != -1 && itemGrid[checkCoord.x][checkCoord.y] != inserter.item) worldCost += 1.0f;

				itemGrid[checkCoord.x][checkCoord.y] = inserter.item;
				itemEndpoints.push_back({ inserter.item, inserter.rate, !inserter.isInput, checkCoord });
			}
		}

		isWorldValid = worldCost == 0.0f;
		worldCalculated = true;
	}

	void calculateHash()
	{
		std::string hashString = "";
		for (const auto& assembler : assemblers)
		{
			hashString += "(" + std::to_string(assembler.item) + "," + std::to_string(assembler.coordinate.x) + "," + std::to_string(assembler.coordinate.y) + ")";
			for (int i = 0; i < 12; i++)
			{
				const InserterInstance& inserter = assembler.inserters[i];
				hashString += "[" + std::to_string(inserter.item) + "," + std::to_string(inserter.isInput) + "]";
			}
		}
		hash = std::hash<std::string>{}(hashString);
	}
};

std::unordered_map<size_t, std::vector<std::pair<PFData, CATEntry>>> PFState::neighbourCache = std::unordered_map<size_t, std::vector<std::pair<PFData, CATEntry>>>();
std::map<size_t, std::shared_ptr<LSState>> LSState::cachedStates = std::map<size_t, std::shared_ptr<LSState>>();
size_t LSState::evaluationCount = 0;
size_t CBPathfinder::evaluationCount = 0;
size_t PFState::evaluationCount = 0;

//                Main Problem Solver
// ================================================

class ProblemSolver
{
public:
	struct ItemInfo
	{
		int item = -1;
		bool isComponent = false;
		float rate = 0.0f;
	};

public:
	static constexpr float MAX_INSERTER_RATE = 4.62f;
	static constexpr float MAX_CONVEYOR_RATE = 45.0f;

	static ProblemSolver solve(const ProblemDefinition& problem, std::shared_ptr<DataLogger> dataLogger = nullptr)
	{
		// Produce a solver object with parameters then solve
		ProblemSolver solver(problem, dataLogger);
		solver.solve();
		return solver;
	}

private:
	const ProblemDefinition& problem;
	std::shared_ptr<DataLogger> dataLogger;
	int componentItemCount = -1;
	int bestRunConfig = -1;
	std::map<int, ItemInfo> baseItemInfos;
	std::map<int, RunConfig> possibleRunConfigs;

	ProblemSolver(const ProblemDefinition& problem, std::shared_ptr<DataLogger> dataLogger = nullptr)
		: problem(problem), dataLogger(dataLogger)
	{}

	// Main logical solve function
	// Run each solve stage sequentially
	void solve()
	{
		LOG(SOLVER, "Solving...\n\n");

		unravelRecipes();
		calculateRunConfigs();
		performSearch();
	}

	// Unravel output item recipe to inputs
	// Keep track of total rates relative to a single output rate
	// Fails early if an item cannot be crafted from inputs
	// Populates this->baseItemInfos and componentItemCount
	void unravelRecipes()
	{
		struct RecipeTrace
		{
			int item;
			float rate;
		};

		// LOGGING: Print out problem input items, output item, and recipes
		LOG(SOLVER, "Stage: unravelRecipes()\n"
			<< "=========================\n\n");
		LOG(SOLVER, "Problem Input Items: \n");
		for (const auto& entry : problem.itemInputs)
		{
			LOG(SOLVER, "- (" << entry.first << ") at (" << entry.second.coordinate.x << ", " << entry.second.coordinate.y << ") at " << entry.second.rate << "/s\n");
		}
		LOG(SOLVER, "\n");
		LOG(SOLVER,
			"Problem Output Item:\n"
			<< "- (" << problem.itemOutput.item << ") at "
			<< "(" << problem.itemOutput.coordinate.x << ", " << problem.itemOutput.coordinate.y << ")\n\n");

		LOG(SOLVER, "Problem Recipes: \n");
		for (const auto& entry : problem.recipes)
		{
			const int item = entry.first;
			const Recipe& recipe = entry.second;
			LOG(SOLVER, "- " << "(" << item << ")*" << recipe.quantity << " at " << recipe.rate << "/s = ");
			for (const auto& ingredent : recipe.ingredients)
			{
				LOG(SOLVER, "(" << ingredent.item << ")*" << ingredent.quantity << " ");
			}
			LOG(SOLVER, "\n");
		}
		LOG(SOLVER, "\n----------\n\n");

		// Keep track of component items and relative rates
		componentItemCount = 0;
		baseItemInfos.clear();
		std::stack<RecipeTrace> traceStack;

		// Output has a recipe so add to stack with relative rate for 1 assembler
		int outputItem = problem.itemOutput.item;
		if (problem.recipes.find(outputItem) != problem.recipes.end())
		{
			const auto& outputRecipe = problem.recipes.at(outputItem);
			traceStack.push({ outputItem, outputRecipe.rate * outputRecipe.quantity });
		}

		// Output is an input so add to stack with no info
		else if (problem.itemInputs.find(outputItem) != problem.itemInputs.end())
			traceStack.push({ outputItem, 0.0f });

		// Output cannot be reached so error
		else throw std::exception(("Output item (" + std::to_string(outputItem) + ") has no recipe nor is an input.").c_str());

		// Process traced items while any left
		while (traceStack.size() > 0)
		{
			const auto current = traceStack.top();
			traceStack.pop();

			// Initialize or update item info for item
			bool isInput = problem.itemInputs.find(current.item) != problem.itemInputs.end();
			if (baseItemInfos.find(current.item) == baseItemInfos.end())
			{
				baseItemInfos[current.item] = { current.item, !isInput, current.rate };
			}
			else baseItemInfos[current.item].rate += current.rate;

			// Skip if is an input
			if (isInput) continue;

			// No recipe therefore impossible problem
			if (problem.recipes.find(current.item) == problem.recipes.end())
				throw std::exception(("- Component item (" + std::to_string(current.item) + ") has no recipe.").c_str());

			// Has a recipe so recurse down
			componentItemCount++;
			const auto& currentRecipe = problem.recipes.at(current.item);
			for (const auto& ingredient : currentRecipe.ingredients)
			{
				traceStack.push({ ingredient.item, ingredient.quantity * current.rate / currentRecipe.quantity });
			}
		}

		LOG(SOLVER, "Solver Items: \n");
		for (const auto& entry : baseItemInfos)
		{
			LOG(SOLVER, "- (" << entry.second.item << ") = "
				<< (entry.second.isComponent ? "Component" : "Input") << " "
				<< "( Rate: " << entry.second.rate << " )\n");
		}
		LOG(SOLVER, "\n\n");
	}

	// Figure out maximum possible output assembers
	// First check the minimum required assemblers can fit
	// Then calculate max possible, and work backwards
	// Populates this->possibleRunConfigs
	// Return maximum run config
	void calculateRunConfigs()
	{
		LOG(SOLVER,
			"Stage: calculateRunConfigs()\n"
			<< "=============================\n\n");

		// Calculate maximum supported output assemblers
		float maxSupported = 0.0f;
		for (const auto& input : problem.itemInputs)
		{
			const auto& inputItem = input.second;
			const auto& itemInfo = baseItemInfos.at(inputItem.item);
			float itemsSupportedCount = inputItem.rate / itemInfo.rate;
			maxSupported = std::max(maxSupported, itemsSupportedCount);
		}
		int maxSupportedCeil = static_cast<int>(std::ceil(maxSupported));

		// Calculate run configs from max supported to 1
		for (int i = maxSupportedCeil; i > 0; i--)
		{
			RunConfig runConfig;
			runConfig.outputAssemblerCount = i;

			// For each item produce an item config
			for (const auto& itemPair : baseItemInfos)
			{
				const ItemInfo& itemInfo = itemPair.second;
				RunConfig::ItemConfig itemConfig{ itemInfo.item };

				// If it is a component item calculate assemblers and inserters counts
				if (itemInfo.isComponent)
				{
					const Recipe& itemRecipe = problem.recipes.at(itemInfo.item);

					float totalRate = itemInfo.rate * i;
					float singleRate = itemRecipe.rate * itemRecipe.quantity;
					int assemblerCount = static_cast<int>(std::ceil(totalRate / singleRate));
					float realSingleRate = totalRate / assemblerCount;
					int outputCount = static_cast<int>(std::ceil(realSingleRate / MAX_INSERTER_RATE));
					float singleOutputRate = realSingleRate / outputCount;

					itemConfig.assemblerCount = assemblerCount;
					itemConfig.outputInserterRequirement = { outputCount, singleOutputRate };

					for (const auto& input : itemRecipe.ingredients)
					{
						float totalInputRate = realSingleRate * (input.quantity / itemRecipe.quantity);
						int inputCount = static_cast<int>(std::ceil(totalInputRate / MAX_INSERTER_RATE));
						float singleInputRate = totalInputRate / inputCount;

						itemConfig.inputInserterRequirements[input.item] = { inputCount, singleInputRate };
					}
				}

				runConfig.itemConfigs[itemInfo.item] = itemConfig;
			}

			possibleRunConfigs[i] = runConfig;
		}

		// LOGGING: Print out run configs
		for (int i = maxSupportedCeil; i > 0; i--)
		{
			const auto& runConfig = possibleRunConfigs[i];
			LOG(SOLVER, "Run Config with output assemblers = " << runConfig.outputAssemblerCount << "\n");
			for (const auto& item : runConfig.itemConfigs)
			{
				if (item.second.assemblerCount == 0)
				{
					LOG(SOLVER, "\tInput item: " << item.first << "\n");
				}
				else
				{
					LOG(SOLVER, "\tComponent item: " << item.first << "\n");
					LOG(SOLVER, "\t  Assemblers: " << item.second.assemblerCount << "\n");
					LOG(SOLVER, "\t  Output inserters / assembler: " << item.second.outputInserterRequirement.count << " @ " << item.second.outputInserterRequirement.rate << "/s\n");
					for (const auto& inputRequirement : item.second.inputInserterRequirements)
					{
						LOG(SOLVER, "\t  Input inserters / assembler for " << inputRequirement.first << ": " << inputRequirement.second.count << " @ " << inputRequirement.second.rate << "/s\n");
					}
				}
			}
			LOG(SOLVER, "\n");
		}

		// Check space requirements for each
		bestRunConfig = -1;
		size_t availableSpace = problem.blueprintWidth * problem.blueprintHeight - problem.itemInputs.size() - 1;
		for (int i = maxSupportedCeil; i > 0; i--)
		{
			const auto& runConfig = possibleRunConfigs[i];
			size_t requiredSpace = 0;
			for (const auto& itemConfig : runConfig.itemConfigs)
			{
				requiredSpace += itemConfig.second.assemblerCount * 9;
				requiredSpace += itemConfig.second.assemblerCount * itemConfig.second.outputInserterRequirement.count * 2;
				for (const auto& inputRequirement : itemConfig.second.inputInserterRequirements)
				{
					requiredSpace += itemConfig.second.assemblerCount * inputRequirement.second.count * 2;
				}
			}

			LOG(SOLVER, "Run config " << i << " required space: " << requiredSpace << " / " << availableSpace << "\n");

			// Have found the highest run config so break out
			if (requiredSpace <= availableSpace)
			{
				bestRunConfig = i;
				break;
			}
		}

		LOG(SOLVER, "Found best run config: " << bestRunConfig << "\n\n");
	}

	// Perform the main search for a solution
	// Top level local search with placement of assemblers / inserters
	// Lower level conflict based search over orderings of paths
	// Bottom level A* to find paths
	void performSearch()
	{
		LOG(SOLVER, "Stage: performSearch()\n"
			<< "=======================\n\n");

		// for (int i = bestRunConfig; i > 0; i--)
		for (int i = 1; i <= bestRunConfig; i++)
		{
			LOG(SOLVER, "--- Evaluating run config " << i << " ---\n\n");

			const RunConfig& runConfig = possibleRunConfigs.at(i);

			LSState::evaluationCount = 0;
			CBPathfinder::evaluationCount = 0;
			PFState::evaluationCount = 0;

			dataLogger->clear();

			std::shared_ptr<LSState> initialState = LSState::createRandom(problem, runConfig);

			#if USE_ANNEALING
			std::shared_ptr<LSState> finalState = ls::simulatedAnnealing(initialState, ANNEALING_TEMP, ANNEALING_COOLING, ANNEALING_ITERATIONS, dataLogger);
			#else
			std::shared_ptr<LSState> finalState = ls::hillClimbing(initialState, HILLCLIMBING_ITERATIONS, logger);
			#endif

			dataLogger->save("rc" + std::to_string(i) + "(r" + std::to_string(SRAND_SEED) + ")", { "Iteration", "Fitness", "Paths" });

			LOG(SOLVER, "Finished evaluation, summary:\n\n");
			LOG(SOLVER, "- LSState Evaluation count: " << LSState::evaluationCount << "\n");
			LOG(SOLVER, "- CBPathfinder Evaluation count: " << CBPathfinder::evaluationCount << "\n");
			LOG(SOLVER, "- PFState Evaluation count: " << PFState::evaluationCount << "\n");
			LOG(SOLVER, "- Final state fitness: " << finalState->getFitness() << "\n\n");
			if (LOG_SOLVER_ENABLED) finalState->print();
		}
	}
};

//                 Example Problems
// ================================================

void solveExampleProblem1()
{
	// Define problem definition
	std::map<int, Recipe> recipes;
	recipes[1] = { 1, 1.0f, { { 0, 1 } } }; // 1*Item0 -> 1*Item1 @ 1/s

	// 5x5 Blueprint
	// - Input Item0 @ 1/s at (0, 1)
	// - Output Item1 @ at (4, 2)
	ProblemDefinition problem = ProblemDefinitionFactory::create()
		->setRecipes(recipes)
		->setSize(5, 5)
		->addInputItem(0, 1.0f, 0, 1)
		->addOutputItem(1, 4, 2)
		->finalise();

	// Run problem solver with given parameters
	std::shared_ptr<DataLogger> dataLogger = std::make_shared<DataLogger>(std::string(DATA_LOGGER_PREFIX) + "p1_", !USE_DATA_LOGGER);
	ProblemSolver solver = ProblemSolver::solve(problem, dataLogger);
}

void solveExampleProblem2()
{
	// Define problem definition
	std::map<int, Recipe> recipes;
	recipes[1] = { 1, 0.5f, { { 0, 1 } } };           // 1*Item0 -> 1*Item1 @ 0.5/s
	recipes[2] = { 1, 0.5f, { { 0, 2 }, { 1, 2 } } }; // 2*Item0 + 2*Item1 -> 1*Item2 @ 0.5/s

	// 10x10 Blueprint
	// - Input Item0 @ 4/s at (0, 1)
	// - Output Item2 @ at (9, 9)
	ProblemDefinition problem = ProblemDefinitionFactory::create()
		->setRecipes(recipes)
		->setSize(10, 10)
		->addInputItem(0, 4.0f, 0, 1)
		->addOutputItem(2, 9, 9)
		->finalise();

	// Run problem solver with given parameters
	std::shared_ptr<DataLogger> dataLogger = std::make_shared<DataLogger>(std::string(DATA_LOGGER_PREFIX) + "p2_", !USE_DATA_LOGGER);
	ProblemSolver solver = ProblemSolver::solve(problem, dataLogger);
}

void solveExampleProblem3()
{
	// Define problem definition
	std::map<int, Recipe> recipes;
	recipes[2] = { 1, 2.0f, { { 0, 1 }, { 1, 2 }} };  // 1*Item0 + 2*Item1 -> 1*Item2 @ 2.0/s
	recipes[3] = { 1, 0.5f, { { 0, 3 }, { 2, 1 } } }; // 3*Item0 + 2*Item1 -> 1*Item3 @ 0.5/s

	// 15x15 Blueprint
	// - Input Item0 @ 2/s at (0, 2)
	// - Input Item1 @ 4/s at (0, 10)
	// - Output Item3 @ at (14, 14)
	ProblemDefinition problem = ProblemDefinitionFactory::create()
		->setRecipes(recipes)
		->setSize(15, 15)
		->addInputItem(0, 2.0f, 0, 2)
		->addInputItem(1, 4.0f, 0, 10)
		->addOutputItem(3, 14, 14)
		->finalise();

	// Run problem solver with given parameters
	std::shared_ptr<DataLogger> dataLogger = std::make_shared<DataLogger>(std::string(DATA_LOGGER_PREFIX) + "p3_", !USE_DATA_LOGGER);
	ProblemSolver solver = ProblemSolver::solve(problem, dataLogger);
}

void checkPathfinding()
{
	// Initialize pathfinding world, constraints, and goal
	const std::vector<std::vector<bool>> blockedGrid = {
		{ 0, 0, 1, 1, 0, 0, 0 },
		{ 0, 0, 1, 1, 0, 0, 0 },
		{ 0, 0, 0, 0, 0, 1, 0 } };
	std::vector<CATEntry> constraints{};
	PFGoal goal{ Coordinate{ 2, 6 }, 0, 0 };

	// Make initial state and perform pathfinding
	auto initialState = new PFState(blockedGrid, constraints, goal, PFData{ Coordinate{ 0, 0 }, BeltType::None, Direction::E });
	auto path = pf::asPathfinding<PFState, PFData>(initialState, true);

	// Print path
	for (const auto& nodeData : path->nodes) nodeData.print();
}

void checkCBPathfinding()
{
	// Define problem definition from example 1
	std::map<int, Recipe> recipes;
	recipes[1] = { 1, 0.5f, { { 0, 1 } } };           // 1*Item0 -> 1*Item1 @ 0.5/s
	recipes[2] = { 1, 0.5f, { { 0, 2 }, { 1, 2 } } }; // 2*Item0 + 2*Item1 -> 1*Item2 @ 0.5/s

	// 10x10 Blueprint
	// - Input Item0 @ 4/s at (0, 1)
	// - Output Item2 @ at (9, 9)
	ProblemDefinition problem = ProblemDefinitionFactory::create()
		->setRecipes(recipes)
		->setSize(10, 10)
		->addInputItem(0, 4.0f, 0, 1)
		->addOutputItem(2, 9, 9)
		->finalise();

	// Setup example run configuration with pre-calculated rates
	RunConfig runConfig;
	runConfig.outputAssemblerCount = 1;
	runConfig.itemConfigs[0] = { 0, 0, 0, 0.0f };
	runConfig.itemConfigs[1] = { 1, 2, 1, 0.5f, { { 0, { 1, 0.5f } } } };
	runConfig.itemConfigs[2] = { 2, 1, 1, 0.5f, { { 0, { 1, 1.0f } }, { 1, { 1, 1.0f }}} };

	// Setup local search state with pre-placed assemblers
	LSState::InserterInstance none{ -1, false };
	std::vector<LSState::AssemblerInstance> assemblers;
	assemblers.push_back({ 1, { 5, 0 }, { none, none, none, none, none, { 1, 0.5f, false }, { 0, 0.5f, true }, none, none, none, none, none } });
	assemblers.push_back({ 1, { 3, 7 }, { none, none, none, { 0, 0.5f, true }, none, none, none, none, none, none, none, { 1, 0.5f, false } } });
	assemblers.push_back({ 2, { 2, 4 }, { none, none, { 0, 1.0f, true }, { 2, 0.5f, false }, none, none, none, none, none, none, { 1, 1.0f, true }, none } });

	// Check that the state is valid
	std::shared_ptr<LSState> state = std::make_shared<LSState>(problem, runConfig, assemblers);
	state->print();

	// Request cost to trigger pathfinding to check it works
	std::cout << "State fitness: " << state->getFitness() << std::endl;
}
