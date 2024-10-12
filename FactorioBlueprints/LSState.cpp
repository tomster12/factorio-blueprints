#include <memory>
#include <windows.h>
#include "LSState.h"
#include "global.h"

std::shared_ptr<LSState> LSState::createRandom(const ProblemDefinition& problem, const RunConfig& runConfig)
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

LSState::LSState(const ProblemDefinition& problem, const RunConfig& runConfig, const std::vector<AssemblerInstance>& assemblers)
	: problem(problem), runConfig(runConfig), assemblers(assemblers)
{
	calculateHash();
}

float LSState::getFitness()
{
	if (costCalculated) return fitness;

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

	Global::evalCountLS++;
	costCalculated = true;
	return fitness;
}

std::vector<std::shared_ptr<LSState>> LSState::getNeighbours()
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

			neighbours.push_back(std::make_shared<LSState>(problem, runConfig, newAssemblers));
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

				neighbours.push_back(std::make_shared<LSState>(problem, runConfig, newAssemblers));
			}
		}
	}

	neighboursCalculated = true;
	return neighbours;
}

size_t LSState::getHash()
{
	return hash;
}

bool LSState::getValid()
{
	calculateWorld();
	return isWorldValid;
}

void LSState::print()
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

std::vector<float> LSState::generateDataLog()
{
	std::vector<float>  log;

	// Log fitness
	log.push_back(fitness);

	// Log number of paths CBS found
	if (pathfinder == nullptr) log.push_back(0.0f);
	else log.push_back((float)pathfinder->getPathsFound());

	return log;
}

void LSState::calculateWorld()
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

void LSState::calculateHash()
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
