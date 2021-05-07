#include "device_manager.h"
#include "exception.h"

#include <eh.h>
#include "defines.h"

#include <utility.h>

std::string decode_exception_pointer(_EXCEPTION_POINTERS *ep)
{
	PEXCEPTION_RECORD rec = ep->ExceptionRecord;

	std::ostringstream oss;
	oss << fmt::format("\ncode:{}, flag:{}, address:{}, param_count:{}\n", rec->ExceptionCode, rec->ExceptionFlags, rec->ExceptionAddress, rec->NumberParameters);

	for (DWORD i = 0; i < rec->NumberParameters; i++)
	{
		oss << fmt::format("param[{}]: {}\n", i, rec->ExceptionInformation[i]);
	}

	return oss.str();
}

void se_translator_function(const unsigned int code, struct _EXCEPTION_POINTERS *ep)
{
	throw ep;
}

symb::scanner::DeviceManager::DeviceManager()
{
	events_.push(rs2::event_information({}, rs2::context().query_devices()));
	handler_ = std::thread([&]() {
		_set_se_translator(se_translator_function);
		SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
		rs2::context ctx;
		ctx.set_devices_changed_callback([&](const rs2::event_information &event_info) {
			std::lock_guard<std::mutex> _(m_);
			events_.push(event_info);
		});
		while (alive_)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			{
				std::lock_guard<std::mutex> _(n_);
				std::atomic<bool> retrying = false;
				rs2::event_information retry_event({}, {});
				while (!events_.empty())
				{
					const rs2::event_information event = retrying ? retry_event : events_.front();
					if (!retrying)
					{
						std::lock_guard<std::mutex> __(m_);
						events_.pop();
					}
					retrying = false;

					// Un-Register Disconnected Devices
					for (std::map<std::string, rs2::device>::iterator itr = devices_.begin(); itr != devices_.end();)
					{
						try
						{
							if (event.was_removed(itr->second))
							{
								const std::string serial = itr->first;
								itr = devices_.erase(itr);
								SPDLOG_DEBUG("Device Manager Un-Registered {}", serial);
							}
							else
							{
								++itr;
							}
						}
						catch (...)
						{
							++itr;
						}
					}

					// Register Newly Connected Devices
					{
						const rs2::device_list &new_devices = event.get_new_devices();
						SPDLOG_DEBUG("{} New devices", new_devices.size());
						if (new_devices.size() > 0)
						{
							std::mutex im;
							std::atomic<unsigned int> count = 0;
							std::vector<std::thread> threads;
							const int start_size = new_devices.size();
							std::lock_guard<std::mutex> __(gse_);
							system_exception_cnt_ = 0;
							for (unsigned int i = 0; i < new_devices.size(); ++i)
							{
								threads.emplace_back(std::thread([i, &count, &im, &new_devices, this, &retrying, start_size]() {
									try
									{
										rs2::device device = new_devices[i];
										if (device)
										{
											try
											{
												const std::string serial = device.get_info(rs2_camera_info::RS2_CAMERA_INFO_SERIAL_NUMBER);
												if (devices_.find(serial) == devices_.end())
												{

													{
														std::lock_guard<std::mutex> _(im);
														devices_.insert(std::make_pair(serial, device));
														fresh_devices_[serial] = true;
													}
													SPDLOG_DEBUG("{:>2}/{} Device Manager Registered {}", ++count, start_size, serial);
												}
											}
											catch (...)
											{
												SPDLOG_ERROR("{:>2}/{} Device Manager doing hardware_reset", ++count, start_size);
												device.hardware_reset();
												throw;
											}
										}
										else
										{
											SPDLOG_ERROR("{:>2}/{} Device Manager Unable to retreive serial number from device [rs2_device is null]", ++count, start_size);
											retrying = true;
										}
									}
									catch (const rs2::error &e)
									{
										SPDLOG_ERROR("{:>2}/{} Device Manager Unable to register device [{}({}): {}]", ++count, start_size, 
											e.get_failed_function(), e.get_failed_args(), e.what());
										if (!symb::scanner::utility::contains(e.what(), "Camera not connected!"))
										{
											retrying = true;
										}
									}
									catch (const std::exception &e)
									{
										SPDLOG_ERROR("{:>2}/{} Device Manager Unable to register device [{}]", ++count, start_size, e.what());
										retrying = true;
									}
									catch (_EXCEPTION_POINTERS *ep)
									{
										{
											std::lock_guard<std::mutex> _(se_);
											system_exception_cnt_ += 1;
										}
										SPDLOG_WARN("{:>2}/{} Device Manager threw system exception: {}", ++count, start_size, decode_exception_pointer(ep));
									}
									catch (...)
									{
										{
											std::lock_guard<std::mutex> _(se_);
											system_exception_cnt_ += 1;
										}
										SPDLOG_WARN("{:>2}/{} Device Manager threw system exception: Unknown", ++count, start_size);
									}
								}));
							}
							for (std::thread &thread : threads)
							{
								thread.join();
							}
						}
					}
					if (!alive_)
					{
						break;
					}
					if (retrying)
					{
						retry_event = event;
					}
				}
				if (reset_all_ || !reset_serials_.empty())
				{
					for (auto &&dev : devices_)
					{
						bool do_reset = reset_all_;
						if (!do_reset)
						{
							do_reset = std::find_if(reset_serials_.begin(), reset_serials_.end(), [&dev](const std::string &serial) { return serial == dev.first; }) != reset_serials_.end();
						}
						if (do_reset)
						{
							try
							{
								const std::string &serial = dev.first;
								try
								{
									for (const rs2::sensor &sensor: dev.second.query_sensors())
									{
										try
										{
											sensor.stop();
										}
										catch (...)
										{
										}
										try
										{
											sensor.close();
										}										
										catch (...)
										{
										}
									}
								}
								catch (...)
								{
								}
								dev.second.hardware_reset();
								SPDLOG_DEBUG("Device Manager Resetting {}", serial);
							}
							catch (...)
							{
							}
						}
					}
					{
						std::lock_guard<std::mutex> __(o_);
						reset_all_ = false;
						reset_serials_.clear();
					}
				}
			}
		}
		devices_.clear();
	});
}

symb::scanner::DeviceManager::~DeviceManager()
{
	if (alive_)
	{
		alive_ = false;
		handler_.join();
	}
}

void symb::scanner::DeviceManager::reset(const std::vector<std::string> &serials)
{
	{
		std::lock_guard<std::mutex> _(n_);
		for (std::map<std::string, rs2::device>::iterator itr = devices_.begin(); itr != devices_.end();)
		{
			if (serials.empty() || (std::find(serials.begin(), serials.end(), itr->first) != serials.end()))
			{
				try
				{
					for (const rs2::sensor &sensor: itr->second.query_sensors())
					{
						sensor.stop();
						sensor.close();
					}
				}
				catch (...)
				{
				}
				const std::string serial = itr->first;
				itr = devices_.erase(itr);
				SPDLOG_WARN("Device Manager Un-Registered {} for Reset", serial);
			}
			else
			{
				++itr;
			}
		}
		events_.push(rs2::event_information({}, rs2::context().query_devices()));
	}
	{
		std::lock_guard<std::mutex> _(o_);
		reset_all_ = serials.empty();
		if (!reset_all_)
		{
			reset_serials_.reserve(reset_serials_.size() + serials.size());
			reset_serials_.insert(reset_serials_.begin(), serials.begin(), serials.end());
		}
	}
	while (reset_all_ || !reset_serials_.empty())
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
}

bool symb::scanner::DeviceManager::is_fresh(const std::string &serial)
{
	try
	{
		return fresh_devices_[serial];
	}
	catch(...)
	{
		return false;
	}
}



bool symb::scanner::DeviceManager::is_connected(const std::string &serial)
{
	std::lock_guard<std::mutex> _(n_);
	for (auto &&device : devices_)
	{
		if (device.first == serial)
		{
			fresh_devices_[serial] = false;
			return true;
		}
	}
	return false;
}

rs2::device &symb::scanner::DeviceManager::get_rs2_device(const std::string &serial)
{
	std::lock_guard<std::mutex> _(n_);
	for (auto &&device : devices_)
	{
		if (device.first == serial)
		{
			fresh_devices_[serial] = false;
			return device.second;
		}
	}
	throw error::RecoverableError(fmt::format("Device Not Connected ({}:{})", FILENAME, __LINE__));
}

void symb::scanner::DeviceManager::stop() {
	if (alive_)
	{
		alive_ = false;
		handler_.join();
	}
}
