#pragma once

#include <functional>

template <typename... Args>
class ListenableEvent
{
public:
	void addListener(const std::function<void(Args...)>& listener)
	{
		listeners.push_back(listener);
	}

	void trigger(Args... args)
	{
		for (const auto& listener : listeners)
		{
			listener(args...);
		}
	}

private:
	std::vector<std::function<void(Args...)>> listeners;
};
