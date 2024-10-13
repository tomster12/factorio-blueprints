#include "PlacementPathfinder.h"
#include "log.h"
#include "macros.h"
#include "types.h"
#include "global.h"

// CAT
// - Contains an accessible table of conflicts
// - Conflicts signify a position and a conflict group
// - Can add and remove paths, check for conflicts, and get conflicting paths

// PFState
// - Represents a state in the pathfinding algorithm
// - Contains a coordinate, type, and direction
// - Can be converted to a CATEntry for conflict checking

// PlacementPathfinder
// - Contains a blocked grid and item endpoints from a PlacementState
// - Does some setup with matching endpoints up to make paths
// - Performs a conflict based search for pathfinding

// CTNode
// - Represents a node in the conflict based search
// - Contains a set of constraints, a set of paths to calculate, a set of calculated paths, and a cost
// - When this is calculated the paths to calculate are resolved in order

// Optimization Notes
// - First step:
//   - performPathfinding() first initializes an empty node with all the paths set to be calculated
//   - The calculateNode() function then calculates the paths in order and updates the node solutions
//   - The solutions are copied into the node CAT and conflicts are found
//   - The node is then added to the open set if valid
// - Now each loop:
//   - Pop a node off stack
//   - Split into 2 new nodes on the found conflict
//   - Copy existing solutions into nodes
//   - Calculate each and add onto stack
//
// - The majority of the time taken is as follows:
//   - 47% Calculating the pathfinding
//   - 35% Copying solutions into CAT

namespace impl
{
	CAT::CAT() {}

	CAT::CAT(const CAT& other)
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

	void CAT::addPath(size_t pathIndex, const CATEntry& entry)
	{
		if (table.find(entry.coordinateHash) == table.end())
		{
			table[entry.coordinateHash] = { {}, {}, {} };
		}

		if (entry.conflictFlags & 0b001) table[entry.coordinateHash][0].insert(pathIndex);
		if (entry.conflictFlags & 0b010) table[entry.coordinateHash][1].insert(pathIndex);
		if (entry.conflictFlags & 0b100) table[entry.coordinateHash][2].insert(pathIndex);
	}

	void CAT::removePath(size_t pathIndex)
	{
		for (auto& entry : table)
		{
			for (auto& conflictSet : entry.second)
			{
				conflictSet.erase(pathIndex);
			}
		}
	}

	bool CAT::checkConflict(const CATEntry& entry) const
	{
		auto found = table.find(entry.coordinateHash);
		if (found == table.end()) return false;

		const auto& conflictSets = found->second;
		if (entry.conflictFlags & 0b001 && conflictSets[0].size() > 1) return true;
		if (entry.conflictFlags & 0b010 && conflictSets[1].size() > 1) return true;
		if (entry.conflictFlags & 0b100 && conflictSets[2].size() > 1) return true;
		return false;
	}

	std::pair<CATEntry, std::set<size_t>> CAT::getConflictingPaths() const
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

	PFState::PFState(const std::vector<std::vector<bool>>& blockedGrid, const std::vector<CATEntry>& constraints, PFGoal goal, PFData data)
		: blockedGrid(blockedGrid), constraints(constraints), goal(goal), pf::State<PFState, PFData>(data, nullptr)
	{}

	PFState::PFState(PFData data, PFState* parent)
		: pf::State<PFState, PFData>(data, parent), blockedGrid(parent->blockedGrid), constraints(parent->constraints), goal(parent->goal)
	{}

	bool PFState::isGoal()
	{
		bool match = true;
		match &= data.coordinate.x == goal.coordinate.x && data.coordinate.y == goal.coordinate.y;
		if (goal.typeFlags > 0) match &= (goal.typeFlags & static_cast<uint8_t>(data.type)) > 0;
		if (goal.directionFlags > 0) match &= (goal.directionFlags & static_cast<uint8_t>(data.direction)) > 0;
		return match;
	}

	float PFState::getFCost()
	{
		calculateCosts();
		return fCost;
	}

	float PFState::getGCost()
	{
		calculateCosts();
		return gCost;
	}

	float PFState::getHCost()
	{
		calculateCosts();
		return hCost;
	}

	std::vector<PFData*> PFState::getNeighbours()
	{
		calculateNeighbourCache();

		// Calculate viable neighbours from cache that arent blocked or conflict
		std::vector<PFData*> neighbours;
		neighbours.reserve(neighbourCache[hash].size());

		for (size_t i = 0; i < neighbourCache[hash].size(); i++)
		{
			PFData& neighbourData = neighbourCache[hash][i].first;
			if (blockedGrid[neighbourData.coordinate.x][neighbourData.coordinate.y] && (neighbourData.type != BeltType::None && neighbourData.type != BeltType::Underground)) continue;

			const CATEntry& neighbourEntry = neighbourCache[hash][i].second;
			if (std::find_if(constraints.begin(), constraints.end(), [&](const auto& constraint) { return checkCATEntryConflict(constraint, neighbourEntry); }) != constraints.end()) continue;

			neighbours.push_back(&neighbourData);
		}
		return neighbours;
	}

	const PFGoal& PFState::getGoal() const
	{
		return goal;
	}

	bool PFState::conflictsWithConstraints() const
	{
		// Check if any entry in constraints conflicts with this state
		for (const CATEntry& entry : constraints)
		{
			if (checkCATEntryConflict(entry, calculateCATEntry(data.coordinate, data.type, data.direction))) return true;
		}
		return false;
	}

	void PFState::calculateCosts()
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

	void PFState::calculateNeighbourCache()
	{
		if (neighbourCache.find(hash) == neighbourCache.end())
		{
			neighbourCache[hash] = std::vector<std::pair<PFData, CATEntry>>();

			if (data.type == BeltType::None)
			{
				for (int i = 0; i < 4; i++)
				{
					Direction newDirection = static_cast<Direction>(1 << i);
					neighbourCache[hash].push_back({ { data.coordinate, BeltType::Conveyor, newDirection }, {} });
					neighbourCache[hash].push_back({ { data.coordinate, BeltType::UndergroundEntrance, newDirection }, {} });
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
					neighbourCache[hash].push_back({ { newCoord, BeltType::Conveyor, newDirection }, {} });
					neighbourCache[hash].push_back({ { newCoord, BeltType::UndergroundEntrance, newDirection }, {} });
				}
			}

			else if (data.type == BeltType::UndergroundEntrance)
			{
				Coordinate offset = dirOffset(data.direction);
				Coordinate newCoord = { data.coordinate.x + offset.x, data.coordinate.y + offset.y };
				if (newCoord.x >= 0 && newCoord.x < blockedGrid.size() && newCoord.y >= 0 && newCoord.y < blockedGrid[0].size())
				{
					neighbourCache[hash].push_back({ { newCoord, BeltType::Underground, data.direction }, {} });
					neighbourCache[hash].push_back({ { newCoord, BeltType::UndergroundExit, data.direction }, {} });
				}
			}

			else if (data.type == BeltType::Underground)
			{
				Coordinate offset = dirOffset(data.direction);
				Coordinate newCoord = { data.coordinate.x + offset.x, data.coordinate.y + offset.y };
				if (newCoord.x >= 0 && newCoord.x < blockedGrid.size() && newCoord.y >= 0 && newCoord.y < blockedGrid[0].size())
				{
					neighbourCache[hash].push_back({ { newCoord, BeltType::Underground, data.direction }, {} });
					neighbourCache[hash].push_back({ { newCoord, BeltType::UndergroundExit, data.direction }, {} });
				}
			}

			else if (data.type == BeltType::UndergroundExit)
			{
				Coordinate offset = dirOffset(data.direction);
				Coordinate newCoord = { data.coordinate.x + offset.x, data.coordinate.y + offset.y };
				if (newCoord.x >= 0 && newCoord.x < blockedGrid.size() && newCoord.y >= 0 && newCoord.y < blockedGrid[0].size())
				{
					neighbourCache[hash].push_back({ { newCoord, BeltType::Conveyor, data.direction }, {} });
					neighbourCache[hash].push_back({ { newCoord, BeltType::UndergroundEntrance, data.direction }, {} });
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
						neighbourCache[hash].push_back({ { newCoord, BeltType::Conveyor, newDirection }, {} });
						neighbourCache[hash].push_back({ { newCoord, BeltType::UndergroundEntrance, newDirection }, {} });
					}
				}
			}

			for (auto& neighbour : neighbourCache[hash])
			{
				neighbour.second = calculateCATEntry(neighbour.first.coordinate, neighbour.first.type, neighbour.first.direction);
			}
		}
	}

	std::unordered_map<size_t, std::vector<std::pair<PFData, CATEntry>>> PFState::neighbourCache = std::unordered_map<size_t, std::vector<std::pair<PFData, CATEntry>>>();

	PlacementPathfinder::PlacementPathfinder(const std::vector<std::vector<bool>>& blockedGrid, const std::vector<ItemEndpoint>& itemEndpoints)
		: blockedGrid(blockedGrid), itemEndpoints(itemEndpoints)
	{}

	float PlacementPathfinder::getFitness()
	{
		if (fitnessCalculated) return fitness;
		Global::evalCountCBP++;

		solve();

		fitnessCalculated = true;
		return fitness;
	}

	size_t PlacementPathfinder::getPathsFound() const
	{
		if (!finalSolutionFound) return 0;
		size_t pathsFound = 0;
		for (const auto& path : finalSolution->solution)
		{
			if (path.second->found) pathsFound++;
		}
		return pathsFound;
	}

	void PlacementPathfinder::print()
	{
		std::cout << "--- CB Pathfinding Search Log ---" << std::endl << std::endl;

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

	void PlacementPathfinder::solve()
	{
		fitness = 0.0f;
		preprocessPaths();
		performPathfinding();
	}

	void PlacementPathfinder::preprocessPaths()
	{
		pathConfigs = std::vector<PathConfig>();

		LOG(CB_PATHS, "  --- CB Item Endpoints ---\n");
		for (size_t i = 0; i < this->itemEndpoints.size(); i++)
		{
			LOG(CB_PATHS, "  Endpoint " << i << ": item " << this->itemEndpoints[i].item
				<< " at (" << this->itemEndpoints[i].coordinate.x << ", " << this->itemEndpoints[i].coordinate.y << ") "
				<< (this->itemEndpoints[i].isSource ? "source" : "destination")
				<< " @ " << this->itemEndpoints[i].rate << "/s\n");
		}
		LOG(CB_PATHS, "  --- CB Endpoint Processing ---\n");

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
				LOG(CB_PATHS, "  Current: index " << currentIndex << ", item " << current.item << " at (" << current.coordinate.x << ", " << current.coordinate.y << ") "
					<< (current.isSource ? "source" : "destination") << " @ " << current.rate << "/s\n");
				if (bestEndpoint.type == ConcreteEndpoint::Type::ITEM)
				{
					const ItemEndpoint& other = this->itemEndpoints[bestEndpoint.index];
					LOG(CB_PATHS, "  Other (world): index " << bestEndpoint.index << " at (" << other.coordinate.x << ", " << other.coordinate.y << ") "
						<< (other.isSource ? "source" : "destination") << " @ " << other.rate << "/s\n");
				}
				else
				{
					const PathConfig& path = this->pathConfigs[bestEndpoint.index];
					float groupSpareRate = this->pathGroupSpareRates[path.pathGroup];
					LOG(CB_PATHS, "  Other (path): index " << bestEndpoint.index << " spare rate @ " << groupSpareRate << "/s\n");
				}
				LOG(CB_PATHS, "  Created path: index " << pathIndex << " group " << pathGroup << " new spare rate @ " << bestSpareRate << "/s\n");

				// Update the spare rate of the path group
				this->pathGroupSpareRates[pathGroup] = bestSpareRate;
			}
		}

		LOG(CB_PATHS, "  --- Final Path configs ---\n");
		for (size_t i = 0; i < this->pathConfigs.size(); i++)
		{
			const PathConfig& path = this->pathConfigs[i];
			LOG(CB_PATHS, "  Path " << i << " (group " << path.pathGroup << "): [");
			if (path.source.type == ConcreteEndpoint::Type::ITEM)
			{
				const ItemEndpoint& source = this->itemEndpoints[path.source.index];
				LOG(CB_PATHS, "  index " << path.source.index << " item " << source.item
					<< " at (" << source.coordinate.x << ", " << source.coordinate.y << ") "
					<< (source.isSource ? "source" : "destination")
					<< " @ " << source.rate << "/s");
			}
			else
			{
				const PathConfig& source = this->pathConfigs[path.source.index];
				LOG(CB_PATHS, "index " << path.source.index << " group " << source.pathGroup << " @ " << this->pathGroupSpareRates[source.pathGroup] << "/s");
			}
			LOG(CB_PATHS, "] to [");
			if (path.destination.type == ConcreteEndpoint::Type::ITEM)
			{
				const ItemEndpoint& destination = this->itemEndpoints[path.destination.index];
				LOG(CB_PATHS, "index " << path.destination.index << " item " << destination.item
					<< " at (" << destination.coordinate.x << ", " << destination.coordinate.y << ") "
					<< (destination.isSource ? "source" : "destination")
					<< " @ " << destination.rate << "/s");
			}
			else
			{
				const PathConfig& destination = this->pathConfigs[path.destination.index];
				LOG(CB_PATHS, "index " << path.destination.index << " group " << destination.pathGroup << " @ " << this->pathGroupSpareRates[destination.pathGroup] << "/s");
			}
			LOG(CB_PATHS, "]\n");
		}
	}

	void PlacementPathfinder::performPathfinding()
	{
		LOG(CB_SEARCH, "  --- CB Pathfinding Search ---\n");

		// NOTE
		// A CTNode will be invalid if a path config cannot be resolved
		// The openSet will eventually be empty if all the CTNodes end up invalid
		// This would happen in a solution with no conflicts but some paths invalid

		// Initialize root with all paths, calculate, exit early if invalid
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

			LOG(CB_SEARCH, "  Open set: " << openSet.size() << ", cost: " << current->cost << ", Constraints: ( ");
			for (const auto& entry : current->constraints)
			{
				LOG(CB_SEARCH, entry.second.size() << " ");
			}
			LOG(CB_SEARCH, ")\n");

			// If no conflict, have found a solution
			if (!current->firstConflict.isConflict)
			{
				this->finalSolutionFound = true;
				this->finalSolution = current;
				break;
			}

			// Create new nodes for each conflicting path
			for (size_t pathIndex : { current->firstConflict.pathA, current->firstConflict.pathB })
			{
				// Initialize next node and copy across values
				auto next = std::make_shared<CTNode>();
				next->isValid = true;
				next->constraints = current->constraints;
				next->solution = current->solution;
				next->cost = current->cost;
				next->pathsToCalculate = { pathIndex };
				next->constraints[pathIndex].push_back(current->firstConflict.catEntry);

				// Calculate the node and add to open set if valid
				calculateNode(next);
				if (next->isValid) openSet.push_back(next);
			}
		}

		// No solution without conflicts found, fitness = 0
		if (!this->finalSolutionFound)
		{
			LOG(CB_SEARCH, "\n  No solution found\n\n");
			fitness = 0.0f;
			return;
		}

		LOG(CB_SEARCH, "  Solution found\n");

		// Solution found, fitness = sum (1 + shortest / real)
		fitness = 0.0f;
		for (size_t i = 0; i < this->pathConfigs.size(); i++)
		{
			if (!this->finalSolution->solution[i]->found)
			{
				LOG(CB_PATHS, "  Path " << i << " in the solution, could not found\n");
				continue;
			}

			// Its possible with underground belts to have a lower cost, but cap it to max
			const auto& first = this->finalSolution->solution[i]->nodes[0];
			const auto& last = this->finalSolution->solution[i]->nodes.back();
			float shortest = pf::ManhattanDistance((float)first.coordinate.x, (float)first.coordinate.y, (float)last.coordinate.x, (float)last.coordinate.y);
			float real = this->finalSolution->solution[i]->cost;
			real = std::max(real, shortest);
			if (shortest == 0) fitness += 2.0f;
			else fitness += (1.0f + shortest / real);

			LOG(CB_PATHS, "  Path " << i << " in the solution, shortest " << shortest << ", real " << real << ", fitness " << (1.0f + shortest / real) << "\n");
		}
	}

	void PlacementPathfinder::calculateNode(std::shared_ptr<CTNode> node)
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

		// NOTE
		// Initialize CAT with already calculated paths and update cost
		// Recreating the CAT each time here is a tradeoff between memory and CPU
		// - Copying the CAT over could actually be slower anyway
		// Potentially additionally need to look at the tradeoff between holding all constraints and copying
		// - These are very small structs so likely not an issue
		CAT cat;
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
			auto source = initPathNodeWithContext(pathIndex, node, cat);
			if (source == nullptr) return;

			node->solution[pathIndex] = pf::asPathfinding<PFState, PFData>(source);
			Global::evalCountPF++;

			node->cost += node->solution[pathIndex]->cost;
			for (const auto& nodeData : node->solution[pathIndex]->nodes) cat.addPath(pathIndex, calculateCATEntry(nodeData.coordinate, nodeData.type, nodeData.direction));
		}

		// Find if there are any conflicts for this node and update
		auto conflict = cat.getConflictingPaths();
		if (conflict.second.size() == 0)
		{
			node->firstConflict = { false };
		}
		else
		{
			auto it = conflict.second.begin();
			size_t pathA = *it;
			size_t pathB = *(++it);
			node->firstConflict = { true, pathA, pathB, conflict.first };
		}

		// Node is valid once all paths found
		node->isValid = true;
	}

	PFState* PlacementPathfinder::initPathNodeWithContext(size_t pathIndex, const std::shared_ptr<CTNode>& node, const CAT& cat)
	{
		// Attempt to resolve the path considering the given CT node
		// Return nullptr if conflicts and constraints prevent resolution
		const PathConfig& path = this->pathConfigs[pathIndex];
		const auto& constraints = node->constraints[pathIndex];

		// Path -> Endpoint, resolve closest source path edge
		if (path.source.type == ConcreteEndpoint::Type::PATH)
		{
			const ItemEndpoint& destinationEndpoint = this->itemEndpoints[path.destination.index];

			const auto destinationCATEntry = calculateCATEntry(destinationEndpoint.coordinate, BeltType::Conveyor, Direction::N);
			if (std::any_of(constraints.begin(), constraints.end(), [&](const CATEntry& entry) { return checkCATEntryConflict(entry, destinationCATEntry); })) return nullptr;
			if (cat.checkConflict(destinationCATEntry)) return nullptr;

			const auto& sourcePath = node->solution[path.source.index];
			auto best = findPathEdgeWithContext(sourcePath, destinationEndpoint.coordinate, node, cat, constraints);
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

		// Endpoint -> Path, resolve closest destination path edge
		if (path.destination.type == ConcreteEndpoint::Type::PATH)
		{
			const ItemEndpoint& sourceEndpoint = this->itemEndpoints[path.source.index];

			const auto sourceCATEntry = calculateCATEntry(sourceEndpoint.coordinate, BeltType::Conveyor, Direction::N);
			if (std::any_of(constraints.begin(), constraints.end(), [&](const CATEntry& entry) { return checkCATEntryConflict(entry, sourceCATEntry); })) return nullptr;
			if (cat.checkConflict(sourceCATEntry)) return nullptr;

			const auto& destinationPath = node->solution[path.destination.index];
			auto best = findPathEdgeWithContext(destinationPath, sourceEndpoint.coordinate, node, cat, constraints);
			if (std::get<0>(best) == false) return nullptr;

			PFGoal goal{
				std::get<1>(best),
				static_cast<uint8_t>(BeltType::Conveyor) |
				static_cast<uint8_t>(BeltType::UndergroundExit),
				static_cast<uint8_t>(dirOpposite(std::get<2>(best)))
			};

			return new PFState(blockedGrid, node->constraints[pathIndex], goal, PFData{ sourceEndpoint.coordinate, BeltType::None, Direction::N });
		}

		// Endpoint -> Endpoint
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

	std::tuple<bool, Coordinate, Direction> PlacementPathfinder::findPathEdgeWithContext(const std::shared_ptr<pf::Path<PFData>>& path, const Coordinate& target, const std::shared_ptr<CTNode>& node, const CAT& cat, const std::vector<CATEntry>& constraints)
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
}
