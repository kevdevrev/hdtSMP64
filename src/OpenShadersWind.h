#pragma once

#include "../extern/OpenShaders/CSinterface001.h"

namespace hdt::OpenShadersWind
{
	/** @brief Resolves the optional wind API after all SKSE plugins have loaded. */
	void Initialize();
	/** @brief Returns the available revision-4 provider, regardless of the user's selection. */
	CSPluginAPI::ICSInterface001* GetInterface();
	/** @brief Returns the provider when available and selected; otherwise null. */
	CSPluginAPI::ICSInterface001* GetActiveInterface();
	/** @brief True when WindField API is available and selected; the master wind switch is separate. */
	bool IsEnabled();
}
