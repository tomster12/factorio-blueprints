#include <memory>
#include <windows.h>
#include "PlacementState.h"
#include "global.h"

// PlacementState
// - Represents a palcement of assemblers and inserters in the world
// - Calculates blocked grid and item endpoints
// - Instantiates a PlacementPathfinder for CB pathfinding

namespace impl
{
	PlacementState* PlacementState::createRandom(const PlacementConfig& config)
	{
		std::vector<AssemblerInstance> assemblerPlacements;

		// For each item type
		for (const auto& itemConfigPair : config.runConfig.itemConfigs)
		{
			const auto& itemConfig = itemConfigPair.second;
			if (itemConfig.assemblerCount == 0) continue;

			// Randomly place assemblers
			for (int i = 0; i < itemConfig.assemblerCount; i++)
			{
				Coordinate coord;
				coord.x = rand() % (config.problem.blueprintWidth - 2);
				coord.y = rand() % (config.problem.blueprintHeight - 2);
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

				assemblerPlacements.push_back(assembler);
			}
		}

		return new PlacementState(config, assemblerPlacements);
	}

	PlacementState::PlacementState(const PlacementConfig& config, const std::vector<AssemblerInstance>& assemblerPlacements)
		: config(config), assemblerPlacements(assemblerPlacements), ls::State<PlacementState>()
	{
		calculateHash();
	}

	PlacementState::~PlacementState()
	{
		delete pathfinder;
	}

	float PlacementState::getFitness()
	{
		if (isFitnessCalculated) return fitness;

		fitness = 0.0f;

		calculateWorld();
		fitness -= worldCost;

		#if USE_PATHFINDING
		if (isWorldValid)
		{
			pathfinder = new PlacementPathfinder(blockedGrid, itemEndpoints);
			fitness += pathfinder->getFitness();
		}
		#endif

		Global::evalCountLS++;
		isFitnessCalculated = true;
		return fitness;
	}

	std::vector<PlacementState*> PlacementState::getNeighbours(ls::StateCache<PlacementState>& cache)
	{
		if (areNeighboursCalculated) return neighbours;

		neighbours = std::vector<PlacementState*>();

		for (int i = 0; i < assemblerPlacements.size(); i++)
		{
			const AssemblerInstance& assembler = assemblerPlacements[i];

			// Neighbour for moving each assembler in each direction
			for (int j = 0; j < 4; j++)
			{
				Direction dir = static_cast<Direction>(1 << j);
				Coordinate offset = dirOffset(dir);
				Coordinate newCoord = { assembler.coordinate.x + offset.x, assembler.coordinate.y + offset.y };
				if (newCoord.x < 0 || newCoord.x >= config.problem.blueprintWidth - 3 || newCoord.y < 0 || newCoord.y >= config.problem.blueprintHeight - 3) continue;

				std::vector<AssemblerInstance> newAssemblerPlacements = assemblerPlacements;
				newAssemblerPlacements[i].coordinate = newCoord;

				PlacementState* neighbour = new PlacementState(config, newAssemblerPlacements);
				neighbours.push_back(cache.getCached(neighbour));
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

					std::vector<AssemblerInstance> newAssemblerPlacements = assemblerPlacements;
					newAssemblerPlacements[i].inserters[j] = InserterInstance{ -1, false };
					newAssemblerPlacements[i].inserters[k] = inserter;

					PlacementState* neighbour = new PlacementState(config, newAssemblerPlacements);
					neighbours.push_back(cache.getCached(neighbour));
				}
			}
		}

		areNeighboursCalculated = true;
		return neighbours;
	}

	void PlacementState::clearNeighbours()
	{
		neighbours.clear();
	}

	bool PlacementState::getValid()
	{
		calculateWorld();
		return isWorldValid;
	}

	void PlacementState::print()
	{
		calculateWorld();

		std::cout << "--- Placement State World Log ---" << std::endl << std::endl;

		HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);

		for (int y = 0; y < config.problem.blueprintHeight; y++)
		{
			for (int x = 0; x < config.problem.blueprintWidth; x++)
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

		std::cout << "--- Placement State Data Log ---" << std::endl << std::endl;

		for (const auto& assembler : assemblerPlacements)
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

	void PlacementState::calculateWorld()
	{
		if (isWorldCalculated) return;

		itemEndpoints = std::vector<ItemEndpoint>();
		blockedGrid = std::vector<std::vector<bool>>(config.problem.blueprintWidth, std::vector<bool>(config.problem.blueprintHeight, false));
		itemGrid = std::vector<std::vector<int>>(config.problem.blueprintWidth, std::vector<int>(config.problem.blueprintHeight, -1));

		// Add input and output items to world
		for (const auto& input : config.problem.itemInputs)
		{
			itemGrid[input.second.coordinate.x][input.second.coordinate.y] = input.first;
			itemEndpoints.push_back({ input.first, input.second.rate, true, input.second.coordinate });
		}
		itemGrid[config.problem.itemOutput.coordinate.x][config.problem.itemOutput.coordinate.y] = config.problem.itemOutput.item;
		itemEndpoints.push_back({ config.problem.itemOutput.item, 0.0f, false, config.problem.itemOutput.coordinate });

		for (const auto& assembler : assemblerPlacements)
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

				if (coord.x < 0 || coord.x >= config.problem.blueprintWidth || coord.y < 0 || coord.y >= config.problem.blueprintHeight)
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

				if (checkCoord.x < 0 || checkCoord.x >= config.problem.blueprintWidth || checkCoord.y < 0 || checkCoord.y >= config.problem.blueprintHeight)
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
		isWorldCalculated = true;
	}

	void PlacementState::calculateHash()
	{
		if (hash != 0) return;

		std::string hashString = "";
		for (const auto& assembler : assemblerPlacements)
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
}
