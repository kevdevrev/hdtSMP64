#include "OpenShadersWind.h"
#include "hdtSkyrimPhysicsWorld.h"

#include <atomic>

namespace hdt::OpenShadersWind
{
	namespace
	{
		std::atomic<CSPluginAPI::ICSInterface001*> windInterface{ nullptr };
	}

	void Initialize()
	{
		CSPluginAPI::CSMessage message;
		const auto messaging = SKSE::GetMessagingInterface();
		CSPluginAPI::ICSInterface001* provider = nullptr;
		if (messaging && messaging->Dispatch(CSPluginAPI::CSMessage::kMessage_GetInterface, &message, sizeof(message), CSPluginAPI::CSPluginName) && message.GetApiFunction) {
			provider = static_cast<CSPluginAPI::ICSInterface001*>(
				message.GetApiFunction(CSPluginAPI::CSInterfaceRevision004));
		}
		windInterface.store(provider, std::memory_order_release);
		logger::info("Open Shaders wind API: {}", provider ? "revision 4 available" : "unavailable; using built-in weather");
	}

	CSPluginAPI::ICSInterface001* GetActiveInterface()
	{
		return SkyrimPhysicsWorld::get()->m_useWindFieldAPI.load(std::memory_order_relaxed) ? GetInterface() : nullptr;
	}

	bool IsEnabled()
	{
		return GetActiveInterface() != nullptr;
	}

	CSPluginAPI::ICSInterface001* GetInterface()
	{
		return windInterface.load(std::memory_order_acquire);
	}
}
