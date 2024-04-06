#pragma once

#include <vector>

#include "Logger.h"

namespace ls
{
	template<typename T>
	class State
	{
	public:
		static std::shared_ptr<T> getCached(std::shared_ptr<T> state)
		{
			size_t hash = state->hash;
			if (cachedStates.find(hash) == cachedStates.end()) cachedStates[hash] = state;
			return cachedStates[hash];
		}

		static void clearCache() { cachedStates = std::map<size_t, std::shared_ptr<T>>(); }

		static size_t getCacheSize() { return cachedStates.size(); }

		bool operator==(State<T>& other) const { return hash == hash; }

		virtual float getFitness() = 0;
		virtual std::vector<std::shared_ptr<T>> getNeighbours() = 0;
		virtual void log(std::vector<float>& logRow) = 0;

	protected:
		static std::map<size_t, std::shared_ptr<T>> cachedStates;
		size_t hash = 0;
	};

	template<typename T>
	std::shared_ptr<T> hillClimbing(std::shared_ptr<T> start, int maxIterations = 100, std::shared_ptr<Logger> logger = nullptr)
	{
		static_assert(std::is_base_of<State<T>, T>::value, "T must be a subclass of State<T>.");

		// Initialize with start state
		T::clearCache();
		T::getCached(start);
		std::shared_ptr<T> current = start;

		#ifdef LOG_LS
		std::cout << "Start, fitness: " << current->getFitness() << std::endl;
		#endif
		if (logger != nullptr)
		{
			std::vector<float> logRow{ 0 };
			current->log(logRow);
			logger->log(logRow);
		}

		// Until max iterations or local maximum
		std::shared_ptr<T> best = start;
		size_t it = 0;
		for (; it < maxIterations; it++)
		{
			// Find best neighbour
			std::shared_ptr<T> best = current;
			for (std::shared_ptr<T>& neighbour : current->getNeighbours())
			{
				if (neighbour->getFitness() > best->getFitness())
				{
					best = neighbour;
				}
			}

			// Found local maximum
			if (best == current)
			{
				#ifdef LOG_LS
				std::cout << "It " << it << ", local maximum" << std::endl;
				#endif
				break;
			}

			// Move to best neighbour
			#ifdef LOG_LS
			std::cout << "It " << it << ", better fitness: " << best->getFitness() << std::endl;
			#endif
			current = best;
			if (logger != nullptr)
			{
				std::vector<float> logRow{ (float)it + 1.0f };
				current->log(logRow);
				logger->log(logRow);
			}

			// Update best
			if (current->getFitness() > best->getFitness()) best = current;
		}

		#ifdef LOG_LS
		std::cout << "Finished " << it << " iterations, fitness: " << best->getFitness() << ", states evaluated: " << T::getCacheSize() << std::endl << std::endl;
		#endif
		return best;
	}

	template<typename T>
	std::shared_ptr<T> simulatedAnnealing(std::shared_ptr<T> start, float temperature = 1000, float coolingRate = 0.005, int maxIterations = 100, std::shared_ptr<Logger> logger = nullptr)
	{
		static_assert(std::is_base_of<State<T>, T>::value, "T must be a subclass of State<T>.");

		T::clearCache();
		T::getCached(start);
		std::shared_ptr<T> current = start;

		#ifdef LOG_LS
		std::cout << "Start, fitness: " << current->getFitness() << std::endl;
		#endif
		if (logger != nullptr)
		{
			std::vector<float> logRow{ 0 };
			current->log(logRow);
			logger->log(logRow);
		}

		// Until max iterations or local maximum
		size_t it = 0;
		std::shared_ptr<T> best = start;
		for (; it < maxIterations && temperature > 0.01f; it++)
		{
			// Find random neighbour
			std::shared_ptr<T> next = current->getNeighbours()[rand() % current->getNeighbours().size()];
			float delta = next->getFitness() - current->getFitness();

			// If better, move to neighbour
			if (delta >= 0)
			{
				current = next;
				#ifdef LOG_LS
				std::cout << "It " << it << ", temperature: " << temperature << ", moving to a better fitness: " << next->getFitness() << std::endl;
				#endif
				if (logger != nullptr)
				{
					std::vector<float> logRow{ (float)it + 1.0f };
					current->log(logRow);
					logger->log(logRow);
				}
			}

			// If worse, accept with probability
			else
			{
				float acceptanceProbability = exp(delta / temperature);
				if ((acceptanceProbability > (rand() / (float)RAND_MAX)))
				{
					current = next;
					#ifdef LOG_LS
					std::cout << "It " << it << ", temperature:" << temperature << ", moving to a worse fitness: " << next->getFitness() << ", chance " << acceptanceProbability << std::endl;
					#endif
					if (logger != nullptr)
					{
						std::vector<float> logRow{ (float)it + 1.0f };
						current->log(logRow);
						logger->log(logRow);
					}
				}
			}

			// Cool system temperature
			temperature *= 1 - coolingRate;

			// Update best
			if (current->getFitness() > best->getFitness())
			{
				best = current;
			}
		}

		#ifdef LOG_LS
		std::cout << "Finished " << it << " iterations, fitness: " << best->getFitness() << ", states seen: " << T::getCacheSize() << std::endl << std::endl;
		#endif

		return best;
	}
}
