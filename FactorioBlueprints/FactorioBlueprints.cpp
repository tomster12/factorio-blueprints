#define LOG
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

#include "LocalSearch.h"
#include "Pathfinding.h"

// ------------------------------------------------

enum class BeltType { None, Inserter, Conveyor, UndergroundEntrance, Underground, UndergroundExit };

enum class Direction { N, S, W, E };

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

// ------------------------------------------------

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
	size_t conflictFlags;
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
			table[entry.coordinateHash] = std::vector<std::set<size_t>>();
			table[entry.coordinateHash].push_back({});
			table[entry.coordinateHash].push_back({});
			table[entry.coordinateHash].push_back({});
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

// ------------------------------------------------

class PFState;

struct PFGoal
{
public:
	PFGoal() : isValid(false) {}

	PFGoal(Coordinate coordinate, std::set<BeltType> types, std::set<Direction> directions)
		: coordinate(coordinate), types(types), directions(directions), isValid(true)
	{}

	bool isValid = false;
	Coordinate coordinate;
	std::set<BeltType> types;
	std::set<Direction> directions;
};

class PFState : public pf::State<PFState>
{
public:
	PFState(
		const std::vector<std::vector<bool>>& blockedGrid, const std::vector<CATEntry>& constraints, PFGoal goal,
		Coordinate coordinate, BeltType type, Direction direction,
		std::shared_ptr<PFState> parent = nullptr)
		: blockedGrid(blockedGrid), constraints(constraints), goal(goal),
		coordinate(coordinate), type(type), direction(direction),
		pf::State<PFState>(parent)
	{
		// Calculate if above ground
		aboveGround = type == BeltType::Conveyor || type == BeltType::UndergroundEntrance || type == BeltType::UndergroundExit;

		// Calculate CAT entry
		catEntry = calculateCATEntry(coordinate, type, direction);
	}

	bool operator==(PFState& other) const
	{
		return coordinate.x == other.coordinate.x && coordinate.y == other.coordinate.y && type == other.type && direction == other.direction;
	}

	CATEntry getCATEntry() const
	{
		return catEntry;
	}

	float getFCost() override
	{
		calculateCosts();
		return gCost + hCost;
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

	std::vector<std::shared_ptr<PFState>> getNeighbours() override
	{
		calculateNeighbours();
		return neighbours;
	}

	bool isAboveGround() const
	{
		return aboveGround;
	}

	Coordinate getCoordinate() const
	{
		return coordinate;
	}

	PFGoal getGoal() const
	{
		return goal;
	}

	bool isGoal() override
	{
		bool match = true;
		match &= coordinate.x == goal.coordinate.x && coordinate.y == goal.coordinate.y;
		if (goal.types.size() > 0) match &= goal.types.find(type) != goal.types.end();
		if (goal.directions.size() > 0) match &= goal.directions.find(direction) != goal.directions.end();
		return match;
	}

	bool conflictsWithConstraints() const
	{
		// Check if any entry in constraints conflicts with this state
		for (const CATEntry& entry : constraints)
		{
			if (checkCATEntryConflict(entry, catEntry)) return true;
		}
		return false;
	}

	void print()
	{
		std::string typeStr = type == BeltType::None ? "None" : type == BeltType::Conveyor ? "Conveyor" : type == BeltType::UndergroundEntrance ? "UndergroundEntrance" : type == BeltType::Underground ? "Underground" : "UndergroundExit";
		std::cout << "State: (" << coordinate.x << ", " << coordinate.y << "), belt type: " << typeStr << ", direction: " << dirString(direction) << std::endl;
	}

private:
	const std::vector<std::vector<bool>>& blockedGrid;
	const std::vector<CATEntry>& constraints;
	PFGoal goal;

	Coordinate coordinate;
	BeltType type;
	Direction direction;
	bool aboveGround;
	CATEntry catEntry;

	bool neighboursCalculated = false;
	bool costCalculated = false;
	std::vector<std::shared_ptr<PFState>> neighbours;
	float gCost = 0.0f;
	float hCost = 0.0f;

	void calculateCosts()
	{
		if (costCalculated) return;

		if (parent == nullptr) gCost = 0.0f;
		else
		{
			float coordDist = pf::EuclideanDistance((float)coordinate.x, (float)coordinate.y, (float)parent->coordinate.x, (float)parent->coordinate.y);
			if ((type == BeltType::Underground) || (parent->type == BeltType::Underground)) coordDist *= 0.5f;
			if (parent->type == BeltType::Inserter) coordDist = 0.0f;
			gCost = parent->gCost + coordDist;
		}

		hCost = pf::EuclideanDistance((float)coordinate.x, (float)coordinate.y, (float)goal.coordinate.x, (float)goal.coordinate.y);

		costCalculated = true;
	}

	void calculateNeighbours()
	{
		if (neighboursCalculated) return;

		neighbours = std::vector<std::shared_ptr<PFState>>();

		if (type == BeltType::None)
		{
			// (4 directions, not blocked) None -> Conveyor | UndergroundEntrance
			for (int i = 0; i < 4; i++)
			{
				if (!blockedGrid[coordinate.x][coordinate.y])
				{
					Direction newDirection = static_cast<Direction>(i);
					neighbours.push_back(std::make_shared<PFState>(blockedGrid, constraints, goal, coordinate, BeltType::Conveyor, newDirection, std::make_shared<PFState>(*this)));
					neighbours.push_back(std::make_shared<PFState>(blockedGrid, constraints, goal, coordinate, BeltType::UndergroundEntrance, newDirection, std::make_shared<PFState>(*this)));
				}
			}
		}

		else if (type == BeltType::Conveyor)
		{
			for (int i = 0; i < 4; i++)
			{
				// (3 directions, not blocked) Conveyor -> Conveyor | UndergroundEntrance
				Direction newDirection = static_cast<Direction>(i);
				if (newDirection == dirOpposite(direction)) continue;

				Coordinate offset = dirOffset(newDirection);
				Coordinate newCoord = { coordinate.x + offset.x, coordinate.y + offset.y };
				if (newCoord.x < 0 || newCoord.x >= blockedGrid.size() || newCoord.y < 0 || newCoord.y >= blockedGrid[0].size()) continue;
				if (blockedGrid[newCoord.x][newCoord.y]) continue;

				neighbours.push_back(std::make_shared<PFState>(blockedGrid, constraints, goal, newCoord, BeltType::Conveyor, newDirection, std::make_shared<PFState>(*this)));
				neighbours.push_back(std::make_shared<PFState>(blockedGrid, constraints, goal, newCoord, BeltType::UndergroundEntrance, newDirection, std::make_shared<PFState>(*this)));
			}
		}

		else if (type == BeltType::UndergroundEntrance)
		{
			// (Forwards) UndergroundEntrance -> Underground
			Coordinate offset = dirOffset(direction);
			Coordinate newCoord = { coordinate.x + offset.x, coordinate.y + offset.y };

			if (newCoord.x >= 0 && newCoord.x < blockedGrid.size() && newCoord.y >= 0 && newCoord.y < blockedGrid[0].size())
			{
				neighbours.push_back(std::make_shared<PFState>(blockedGrid, constraints, goal, newCoord, BeltType::Underground, direction, std::make_shared<PFState>(*this)));

				// (Forwards, not blocked) : UndergroundEntrance -> UndergroundExit
				if (!blockedGrid[newCoord.x][newCoord.y])
				{
					neighbours.push_back(std::make_shared<PFState>(blockedGrid, constraints, goal, newCoord, BeltType::UndergroundExit, direction, std::make_shared<PFState>(*this)));
				}
			}
		}

		else if (type == BeltType::Underground)
		{
			// (Forwards) Underground -> Underground
			Coordinate offset = dirOffset(direction);
			Coordinate newCoord = { coordinate.x + offset.x, coordinate.y + offset.y };

			if (newCoord.x >= 0 && newCoord.x < blockedGrid.size() && newCoord.y >= 0 && newCoord.y < blockedGrid[0].size())
			{
				neighbours.push_back(std::make_shared<PFState>(blockedGrid, constraints, goal, newCoord, BeltType::Underground, direction, std::make_shared<PFState>(*this)));

				// (Forwards, not blocked) : Underground -> UndergroundExit
				if (!blockedGrid[newCoord.x][newCoord.y])
				{
					neighbours.push_back(std::make_shared<PFState>(blockedGrid, constraints, goal, newCoord, BeltType::UndergroundExit, direction, std::make_shared<PFState>(*this)));
				}
			}
		}

		else if (type == BeltType::UndergroundExit)
		{
			// (Forwards) UndergroundExit -> Conveyor | UndergroundEntrance
			Coordinate offset = dirOffset(direction);
			Coordinate newCoord = { coordinate.x + offset.x, coordinate.y + offset.y };

			if (newCoord.x >= 0 && newCoord.x < blockedGrid.size() && newCoord.y >= 0 && newCoord.y < blockedGrid[0].size() && !blockedGrid[newCoord.x][newCoord.y])
			{
				neighbours.push_back(std::make_shared<PFState>(blockedGrid, constraints, goal, newCoord, BeltType::Conveyor, direction, std::make_shared<PFState>(*this)));
				neighbours.push_back(std::make_shared<PFState>(blockedGrid, constraints, goal, newCoord, BeltType::UndergroundEntrance, direction, std::make_shared<PFState>(*this)));
			}
		}

		else if (type == BeltType::Inserter)
		{
			// (Forwards) Inserter -> Conveyor | UndergroundEntrance
			Coordinate offset = dirOffset(direction);
			Coordinate newCoord = { coordinate.x + offset.x, coordinate.y + offset.y };

			if (newCoord.x >= 0 && newCoord.x < blockedGrid.size() && newCoord.y >= 0 && newCoord.y < blockedGrid[0].size() && !blockedGrid[newCoord.x][newCoord.y])
			{
				for (int i = 0; i < 4; i++)
				{
					Direction newDir = Direction(i);
					if (direction != dirOpposite(newDir))
					{
						neighbours.push_back(std::make_shared<PFState>(blockedGrid, constraints, goal, newCoord, BeltType::Conveyor, newDir, std::make_shared<PFState>(*this)));
						neighbours.push_back(std::make_shared<PFState>(blockedGrid, constraints, goal, newCoord, BeltType::UndergroundEntrance, newDir, std::make_shared<PFState>(*this)));
					}
				}
			}
		}

		// Filter neighbours in place based on constraints
		neighbours.erase(std::remove_if(neighbours.begin(), neighbours.end(), [&](std::shared_ptr<PFState> state) { return state->conflictsWithConstraints(); }), neighbours.end());
		neighboursCalculated = true;
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
	struct CTNode
	{
		bool isValid = true;
		std::vector<size_t> pathsToCalculate{};
		std::map<size_t, pf::Path<PFState>> calculatedPaths{};

		ConflictAvoidanceTable cat;
		std::map<size_t, std::vector<CATEntry>> constraints{};
		std::map<size_t, pf::Path<PFState>*> solution{};
		float cost = 0.0f;
	};

	struct CTConflict
	{
		bool isConflict;
		size_t pathA;
		size_t pathB;
		CATEntry catEntry;
	};

	CBPathfinder(const std::vector<std::vector<bool>>& blockedGrid, const std::vector<ItemEndpoint>& itemEndpoints)
		: blockedGrid(blockedGrid), itemEndpoints(itemEndpoints)
	{}

	float getFitness()
	{
		if (fitnessCalculated) return fitness;
		solve();
		fitnessCalculated = true;
		return fitness;
	}

private:
	const std::vector<std::vector<bool>>& blockedGrid;
	const std::vector<ItemEndpoint>& itemEndpoints;

	size_t currentPathGroup = 0;
	std::map<size_t, float> pathGroupSpareRates;
	std::vector<PathConfig> pathConfigs;
	CTNode finalSolution;
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

		#ifdef LOG
		// Print out path ends
		for (size_t i = 0; i < this->itemEndpoints.size(); i++)
		{
			const ItemEndpoint& endpoint = this->itemEndpoints[i];
			std::cout << "Endpoint " << i << ": item " << endpoint.item
				<< " at (" << endpoint.coordinate.x << ", " << endpoint.coordinate.y << ") "
				<< (endpoint.isSource ? "source" : "destination")
				<< " @ " << endpoint.rate << "/s" << std::endl;
		}
		std::cout << std::endl;
		#endif

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

				#ifdef LOG
				// Log picked choices
				std::cout << "Current: index " << currentIndex << ", item " << current.item << " at (" << current.coordinate.x << ", " << current.coordinate.y << ") "
					<< (current.isSource ? "source" : "destination") << " @ " << current.rate << "/s" << std::endl;

				if (bestEndpoint.type == ConcreteEndpoint::Type::ITEM)
				{
					const ItemEndpoint& other = this->itemEndpoints[bestEndpoint.index];
					std::cout << "Other (world): index " << bestEndpoint.index << " at (" << other.coordinate.x << ", " << other.coordinate.y << ") "
						<< (other.isSource ? "source" : "destination") << " @ " << other.rate << "/s" << std::endl;
				}
				else
				{
					const PathConfig& path = this->pathConfigs[bestEndpoint.index];
					float groupSpareRate = this->pathGroupSpareRates[path.pathGroup];
					std::cout << "Other (path): index " << bestEndpoint.index << " spare rate @ " << groupSpareRate << "/s" << std::endl;
				}

				// Log created path
				std::cout << "Created path: index " << pathIndex << " group " << pathGroup << " new spare rate @ " << bestSpareRate << "/s" << std::endl << std::endl;
				#endif

				// Update the spare rate of the path group
				this->pathGroupSpareRates[pathGroup] = bestSpareRate;
			}
		}
	}

	void performPathfinding()
	{
		#ifdef LOG
		// Print out paths
		for (size_t i = 0; i < this->pathConfigs.size(); i++)
		{
			const PathConfig& path = this->pathConfigs[i];
			std::cout << "Path " << i << ": group " << path.pathGroup << " from [";
			if (path.source.type == ConcreteEndpoint::Type::ITEM)
			{
				const ItemEndpoint& source = this->itemEndpoints[path.source.index];
				std::cout << "index " << path.source.index << " item " << source.item
					<< " at (" << source.coordinate.x << ", " << source.coordinate.y << ") "
					<< (source.isSource ? "source" : "destination")
					<< " @ " << source.rate << "/s";
			}
			else
			{
				const PathConfig& source = this->pathConfigs[path.source.index];
				std::cout << "index " << path.source.index << " group " << source.pathGroup << " @ " << this->pathGroupSpareRates[source.pathGroup] << "/s";
			}
			std::cout << "] to [";
			if (path.destination.type == ConcreteEndpoint::Type::ITEM)
			{
				const ItemEndpoint& destination = this->itemEndpoints[path.destination.index];
				std::cout << "index " << path.destination.index << " item " << destination.item
					<< " at (" << destination.coordinate.x << ", " << destination.coordinate.y << ") "
					<< (destination.isSource ? "source" : "destination")
					<< " @ " << destination.rate << "/s";
			}
			else
			{
				const PathConfig& destination = this->pathConfigs[path.destination.index];
				std::cout << "index " << path.destination.index << " group " << destination.pathGroup << " @ " << this->pathGroupSpareRates[destination.pathGroup] << "/s";
			}
			std::cout << "]" << std::endl;
		}
		std::cout << std::endl;
		#endif

		std::vector<CTNode> nodes;
		std::vector<CTNode*> openSet;

		// Create default empty root node
		nodes.push_back({});
		CTNode* root = &nodes.back();
		root->isValid = true;

		// Add all paths to calculate
		for (size_t i = 0; i < this->pathConfigs.size(); i++) root->pathsToCalculate.push_back(i);

		// Calculate and early exit if invalid
		calculateNode(root);
		if (!root->isValid)
		{
			fitness = 0.0f;
			return;
		}

		openSet.push_back(root);

		// While there are nodes to process
		bool foundSolution = false;
		while (openSet.size() > 0)
		{
			// Pop the node with the lowest cost
			CTNode* current = openSet[0];
			for (CTNode* node : openSet)
			{
				if (node->cost < current->cost) current = node;
			}
			openSet.erase(std::remove(openSet.begin(), openSet.end(), current), openSet.end());

			// Validate to find conflicts
			CTConflict conflict = findConflict(current);

			// If no conflict, have found a solution
			if (!conflict.isConflict)
			{
				foundSolution = true;
				this->finalSolution = std::move(*current);
				break;
			}

			// Create new nodes for each conflicting path
			for (size_t pathIndex : { conflict.pathA, conflict.pathB })
			{
				nodes.push_back({});
				CTNode* newNode = &nodes.back();

				// Initialize node
				newNode->isValid = true;
				newNode->pathsToCalculate = { pathIndex };
				newNode->cat = current->cat;
				newNode->constraints = {};
				for (const auto& entry : current->constraints)
				{
					newNode->constraints[entry.first] = entry.second;
				}
				newNode->solution = current->solution;
				newNode->cost = current->cost;

				// Add in the conflict
				newNode->constraints[pathIndex].push_back(conflict.catEntry);

				// Validate and add to open set if valid
				calculateNode(newNode);
				if (newNode->isValid)
				{
					openSet.push_back(newNode);
				}
			}
		}

		// No solution found
		if (!foundSolution)
		{
			fitness = 0.0f;
			return;
		}

		// Solution found so update fitness
		fitness = 0.0f;
		for (size_t i = 0; i < this->pathConfigs.size(); i++)
		{
			if (!this->finalSolution.solution[i]->found) continue;
			float shortest = this->finalSolution.solution[i]->nodes[0]->getHCost();
			float real = this->finalSolution.solution[i]->cost;
			fitness += 1.0f + shortest / real;
		}
	}

	void calculateNode(CTNode* node)
	{
		// Exit early if no paths to calculate
		if (node->pathsToCalculate.size() == 0) return;
		node->isValid = false;

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

		// Remove all paths from the CAT
		for (size_t pathIndex : node->pathsToCalculate)
		{
			node->cat.removePath(pathIndex);
		}

		// Calculate each path
		for (size_t pathIndex : calculateOrder)
		{
			const PathConfig& path = this->pathConfigs[pathIndex];

			// Resolve source and destination
			auto source = resolvePathConfig(node, pathIndex);
			if (source == nullptr) return;

			// Perform pathfinding
			node->calculatedPaths[pathIndex] = pf::asPathfinding<PFState>(source, true);
			node->solution[pathIndex] = &node->calculatedPaths[pathIndex];

			// Add path to CAT
			for (const auto& state : node->calculatedPaths[pathIndex].nodes)
			{
				node->cat.addPath(pathIndex, state->getCATEntry());
			}
		}

		// Node is valid so sum individual path costs
		node->isValid = true;
		node->cost = 0.0f;
		for (const auto path : node->solution)
		{
			node->cost += path.second->cost;
		}
	}

	std::shared_ptr<PFState> resolvePathConfig(CTNode* node, size_t pathIndex)
	{
		const PathConfig& path = this->pathConfigs[pathIndex];

		// Position of destination -> goal, resolve closest source path edge -> PFState
		if (path.source.type == ConcreteEndpoint::Type::PATH)
		{
			const pf::Path<PFState>& sourcePath = *(node->solution[path.source.index]);
			const ItemEndpoint& destinationEndpoint = this->itemEndpoints[path.destination.index];

			auto best = resolveBestPathEdge(node, sourcePath, destinationEndpoint.coordinate, true);
			if (best.first.x == -1) return nullptr;

			PFGoal goal{ destinationEndpoint.coordinate, { BeltType::Conveyor, BeltType::UndergroundEntrance, BeltType::UndergroundExit }, {} };
			return std::make_shared<PFState>(blockedGrid, node->constraints[pathIndex], goal, best.first, BeltType::Inserter, best.second);
		}

		// Position of source -> PFState, resolve closest destination path edge -> goal
		if (path.destination.type == ConcreteEndpoint::Type::PATH)
		{
			const ItemEndpoint& sourceEndpoint = this->itemEndpoints[path.source.index];
			const pf::Path<PFState>& destinationPath = *node->solution[path.destination.index];

			auto best = resolveBestPathEdge(node, destinationPath, sourceEndpoint.coordinate, false);
			if (best.first.x == -1) return nullptr;

			PFGoal goal{ best.first, { BeltType::Conveyor, BeltType::UndergroundEntrance, BeltType::UndergroundExit }, { dirOpposite(best.second) } };
			return std::make_shared<PFState>(blockedGrid, node->constraints[pathIndex], goal, sourceEndpoint.coordinate, BeltType::None, Direction::N);
		}

		// Position of source -> PFState, position of destination -> goal
		const ItemEndpoint& sourceEndpoint = this->itemEndpoints[path.source.index];
		const ItemEndpoint& destinationEndpoint = this->itemEndpoints[path.destination.index];

		PFGoal goal{ destinationEndpoint.coordinate, { BeltType::Conveyor, BeltType::UndergroundEntrance, BeltType::UndergroundExit}, {} };
		return std::make_shared<PFState>(blockedGrid, node->constraints[pathIndex], goal, sourceEndpoint.coordinate, BeltType::None, Direction::N);
	}

	std::pair<Coordinate, Direction> resolveBestPathEdge(const CTNode* node, const pf::Path<PFState>& path, Coordinate target, bool needExtraSpace = false)
	{
		// Look for the closest edge of the path to the target
		float bestDistance = std::numeric_limits<float>::max();
		Coordinate bestCoord = { -1, -1 };
		Direction bestDir = Direction::N;

		// For each above ground node in the path, check the 4 directions
		for (size_t i = 0; i < path.nodes.size(); i++)
		{
			const auto& pfNode = path.nodes[i];
			if (pfNode->isAboveGround())
			{
				Coordinate coord = pfNode->getCoordinate();
				for (int j = 0; j < 4; j++)
				{
					Direction dir = static_cast<Direction>(j);
					Coordinate offset = dirOffset(dir);
					Coordinate newCoord = { coord.x + offset.x, coord.y + offset.y };
					CATEntry catEntry = calculateCATEntry(newCoord, BeltType::Conveyor, dir);

					// Only if not blocked, constrained, or have conflicting CAT entries
					if (newCoord.x < 0 || newCoord.x >= blockedGrid.size() || newCoord.y < 0 || newCoord.y >= blockedGrid[0].size()) continue;
					if (blockedGrid[newCoord.x][newCoord.y]) continue;
					if (node->cat.checkConflict(catEntry)) continue;

					// If need extra space, check the extra space is also valid
					if (needExtraSpace)
					{
						Coordinate extraCoord = { newCoord.x + offset.x, newCoord.y + offset.y };
						CATEntry extraCatEntry = calculateCATEntry(extraCoord, BeltType::Conveyor, dir);

						if (extraCoord.x < 0 || extraCoord.x >= blockedGrid.size() || extraCoord.y < 0 || extraCoord.y >= blockedGrid[0].size()) continue;
						if (blockedGrid[extraCoord.x][extraCoord.y]) continue;
						if (node->cat.checkConflict(extraCatEntry)) continue;
					}

					// Check distance to target
					float distance = pf::EuclideanDistance((float)newCoord.x, (float)newCoord.y, (float)target.x, (float)target.y);
					if (distance < bestDistance)
					{
						bestDistance = distance;
						bestCoord = newCoord;
						bestDir = dir;
					}
				}
			}
		}

		return { bestCoord, bestDir };
	}

	CTConflict findConflict(CTNode* node)
	{
		auto conflict = node->cat.getConflictingPaths();
		const CATEntry& catEntry = conflict.first;
		const auto& conflictingPaths = conflict.second;

		// No conflicting paths
		if (conflictingPaths.size() == 0) return { false };
		assert(conflictingPaths.size() >= 2);

		// Grab first 2 conflicting paths and return
		auto it = conflictingPaths.begin();
		size_t pathA = *it;
		size_t pathB = *(++it);
		return { true, pathA, pathB, catEntry };
	}
};

class LSState : public ls::State<LSState>
{
public:
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

		fitness = 0.0f;

		calculateWorld();
		fitness -= worldCost;

		if (isWorldValid)
		{
			pathfinding = std::make_shared<CBPathfinder>(blockedGrid, itemEndpoints);
			fitness += pathfinding->getFitness();
		}

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
				Direction dir = static_cast<Direction>(j);
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
	std::shared_ptr<CBPathfinder> pathfinding;
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

std::map<size_t, std::shared_ptr<LSState>> LSState::cachedStates = std::map<size_t, std::shared_ptr<LSState>>();

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

	static ProblemSolver solve(const ProblemDefinition& problem)
	{
		// Produce a solver object with parameters then solve
		ProblemSolver solver(problem);
		solver.solve();
		return solver;
	}

private:
	const ProblemDefinition& problem;
	int componentItemCount = -1;
	int bestRunConfig = -1;
	std::map<int, ItemInfo> baseItemInfos;
	std::map<int, RunConfig> possibleRunConfigs;

	ProblemSolver(const ProblemDefinition& problem)
		: problem(problem)
	{}

	// Main logical solve function
	// Run each solve stage sequentially
	void solve()
	{
		#ifdef LOG
		std::cout << "Solving..." << std::endl << std::endl;
		#endif

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

		#ifdef LOG
		std::cout
			<< "Stage: unravelRecipes()" << std::endl
			<< "=========================" << std::endl
			<< std::endl;

		std::cout << "Problem Input Items: " << std::endl;
		for (const auto& entry : problem.itemInputs)
		{
			std::cout << "- (" << entry.first << ") at (" << entry.second.coordinate.x << ", " << entry.second.coordinate.y << ") at " << entry.second.rate << "/s" << std::endl;
		}
		std::cout << std::endl;

		std::cout
			<< "Problem Output Item:" << std::endl
			<< "- (" << problem.itemOutput.item << ") at (" << problem.itemOutput.coordinate.x << ", " << problem.itemOutput.coordinate.y << ")"
			<< std::endl << std::endl;

		std::cout << "Problem Recipes: " << std::endl;
		for (const auto& entry : problem.recipes)
		{
			const int item = entry.first;
			const Recipe& recipe = entry.second;
			std::cout << "- " << "(" << item << ")*" << recipe.quantity << " at " << recipe.rate << "/s = ";
			for (const auto& ingredent : recipe.ingredients)
			{
				std::cout << "(" << ingredent.item << ")*" << ingredent.quantity << " ";
			}
			std::cout << std::endl;
		}

		std::cout << std::endl << "----------" << std::endl << std::endl;
		#endif

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

		#ifdef LOG
		std::cout << "Solver Items: " << std::endl;
		for (const auto& entry : baseItemInfos)
		{
			std::cout << "- (" << entry.second.item << ") = "
				<< (entry.second.isComponent ? "Component" : "Input") << " "
				<< "( Rate: " << entry.second.rate << " )"
				<< std::endl;
		}
		std::cout << std::endl << std::endl;
		#endif
	}

	// Figure out maximum possible output assembers
	// First check the minimum required assemblers can fit
	// Then calculate max possible, and work backwards
	// Populates this->possibleRunConfigs
	// Return maximum run config
	void calculateRunConfigs()
	{
		#ifdef LOG
		std::cout
			<< "Stage: calculateRunConfigs()" << std::endl
			<< "=============================" << std::endl
			<< std::endl;
		#endif

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

		#ifdef LOG
		for (int i = maxSupportedCeil; i > 0; i--)
		{
			const auto& runConfig = possibleRunConfigs[i];
			std::cout << "Run Config with output assemblers = " << runConfig.outputAssemblerCount << std::endl;
			for (const auto& item : runConfig.itemConfigs)
			{
				if (item.second.assemblerCount == 0)
				{
					std::cout << "\tInput item: " << item.first << std::endl;
				}
				else
				{
					std::cout << "\tComponent item: " << item.first << std::endl;
					std::cout << "\t  Assemblers: " << item.second.assemblerCount << std::endl;
					std::cout << "\t  Output inserters / assembler: " << item.second.outputInserterRequirement.count << " @ " << item.second.outputInserterRequirement.rate << "/s" << std::endl;
					for (const auto& inputRequirement : item.second.inputInserterRequirements)
					{
						std::cout << "\t  Input inserters / assembler for " << inputRequirement.first << ": " << inputRequirement.second.count << " @ " << inputRequirement.second.rate << "/s" << std::endl;
					}
				}
			}
			std::cout << std::endl;
		}
		#endif

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

			#ifdef LOG
			std::cout << "Run config " << i << " required space: " << requiredSpace << " / " << availableSpace << std::endl;
			#endif

			// Have found the highest run config so break out
			if (requiredSpace <= availableSpace)
			{
				bestRunConfig = i;
				break;
			}
		}

		#ifdef LOG
		std::cout << "Found best run config: " << bestRunConfig << std::endl << std::endl;
		#endif
	}

	// Perform the main search for a solution
	// Top level local search with placement of assemblers / inserters
	// Lower level conflict based search over orderings of paths
	// Bottom level A* to find paths
	void performSearch()
	{
		#ifdef LOG
		std::cout
			<< "Stage: performSearch()" << std::endl
			<< "=======================" << std::endl
			<< std::endl;
		#endif

		for (int i = bestRunConfig; i > 0; i--)
		{
			const RunConfig& runConfig = possibleRunConfigs.at(i);

			std::shared_ptr<LSState> initialState = LSState::createRandom(problem, runConfig);
			std::shared_ptr<LSState> finalState = ls::simulatedAnnealing(initialState, 2.0f, 0.004f, 1000);

			#ifdef LOG
			std::cout << "Run config " << i << " final state fitness: " << finalState->getFitness() << std::endl << std::endl;
			finalState->print();
			#endif
		}
	}
};

// ------------------------------------------------

void solveExampleProblem1()
{
	// Define problem definition
	std::map<int, Recipe> recipes;
	recipes[1] = { 1, 0.5f, { { 0, 1 } } };
	recipes[2] = { 1, 0.5f, { { 0, 2 }, { 1, 2 } } };

	ProblemDefinition problem = ProblemDefinitionFactory::create()
		->setRecipes(recipes)
		->setSize(10, 10)
		->addInputItem(0, 4.0f, 0, 1)
		->addOutputItem(2, 9, 9)
		->finalise();

	// Run problem solver with given parameters
	ProblemSolver solver = ProblemSolver::solve(problem);
}

void checkPathfinding()
{
	// Initialize pathfinding world, constraints, and goal
	const std::vector<std::vector<bool>> blockedGrid = {
		{ false, false, true, true, false, false, false },
		{ false, false, true, true, false, false, false },
		{ false, false, false, false, false, true, false } };
	std::vector<CATEntry> constraints{};
	PFGoal goal{ Coordinate{ 2, 6 }, {}, {} };

	// Make initial state and perform pathfinding
	std::shared_ptr<PFState> initialState = std::make_shared<PFState>(blockedGrid, constraints, goal, Coordinate{ 0, 0 }, BeltType::None, Direction::E);
	pf::Path<PFState> path = pf::asPathfinding(initialState, true);

	// Print path
	for (std::shared_ptr<PFState> state : path.nodes) state->print();
}

void checkCBPathfinding()
{
	// Define problem definition from example 1
	std::map<int, Recipe> recipes;
	recipes[1] = { 1, 0.5f, { { 0, 1 } } };
	recipes[2] = { 1, 0.5f, { { 0, 2 }, { 1, 2 } } };

	ProblemDefinition problem = ProblemDefinitionFactory::create()
		->setRecipes(recipes)
		->setSize(10, 10)
		->addInputItem(0, 4.0f, 0, 1)
		->addOutputItem(2, 9, 9)
		->finalise();

	// Setup example local search state
	RunConfig runConfig;
	runConfig.outputAssemblerCount = 1;
	runConfig.itemConfigs[0] = { 0, 0, 0, 0.0f };
	runConfig.itemConfigs[1] = { 1, 2, 1, 0.5f, { { 0, { 1, 0.5f } } } };
	runConfig.itemConfigs[2] = { 2, 1, 1, 0.5f, { { 0, { 1, 1.0f } }, { 1, { 1, 1.0f }}} };

	LSState::InserterInstance none{ -1, false };
	std::vector<LSState::AssemblerInstance> assemblers;
	assemblers.push_back({ 1, { 5, 0 }, { none, none, none, none, none, { 1, 0.5f, false }, { 0, 0.5f, true }, none, none, none, none, none } });
	assemblers.push_back({ 1, { 3, 7 }, { none, none, none, { 0, 0.5f, true }, none, none, none, none, none, none, none, { 1, 0.5f, false } } });
	assemblers.push_back({ 2, { 2, 4 }, { none, none, { 0, 1.0f, true }, { 2, 0.5f, false }, none, none, none, none, none, none, { 1, 1.0f, true }, none } });

	std::shared_ptr<LSState> state = std::make_shared<LSState>(problem, runConfig, assemblers);
	state->print();

	// Request cost to trigger pathfinding
	std::cout << "State fitness: " << state->getFitness() << std::endl;
}

int main()
{
	srand(0);
	checkCBPathfinding();
}
