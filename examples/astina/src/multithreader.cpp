#include "helper/multithreader.h"
#include "helper/camera_info_with_data.h"
#include <future>

#include "defines.h"

symb::scanner::helper::MultiThreader::MultiThreader(std::vector<std::shared_ptr<symb::scanner::helper::CameraInfoWithData>> &objs,
													std::function<void(std::shared_ptr<symb::scanner::helper::CameraInfoWithData>)> &lambda,
													const std::chrono::seconds &timeout)
{
	const bool timeout_enabled = timeout.count() > 0;
	std::vector<std::thread> threads;
	threads.reserve(objs.size());
	std::mutex m_, n_;
	for (const std::shared_ptr<symb::scanner::helper::CameraInfoWithData> &obj : objs)
	{
		threads.emplace_back(std::thread([&]() {
			std::mutex m, n;
			std::condition_variable cv;
			std::atomic<bool> *timed_out = nullptr;
			std::thread t([&]() {
				std::unique_lock<std::mutex> ul(n);
				const std::shared_ptr<std::atomic<bool>> timed_out_ = std::make_shared<std::atomic<bool>>(false);
				timed_out = timed_out_.get();
				ul.unlock();
				try
				{
					lambda(obj);
					try
					{
						if (!*timed_out_)
						{
							std::lock_guard<std::mutex> _(n_);
							success.push_back(obj);
						}
					}
					catch (...)
					{
					}
				}
				catch (const error::Error &e)
				{
					if (!*timed_out_)
					{
						std::lock_guard<std::mutex> _(m_);
						fail.emplace_back(obj);
						errors.emplace_back(std::make_pair(obj, e));
					}
				}
				catch (const rs2::error &e)
				{
					if (!*timed_out_)
					{
						std::lock_guard<std::mutex> _(m_);
						fail.emplace_back(obj);
						errors.emplace_back(std::make_pair(obj, error::UnknownError(fmt::format("{}({}): {} ({}:{})", e.get_failed_function(), e.get_failed_args(), e.what(), FILENAME, __LINE__))));
					}
				}
				catch (const std::exception &e)
				{
					if (!*timed_out_)
					{
						std::lock_guard<std::mutex> _(m_);
						fail.emplace_back(obj);
						errors.emplace_back(std::make_pair(obj, error::UnknownError(fmt::format("{} ({}:{})", e.what(), FILENAME, __LINE__))));
					}
				}
				catch (...)
				{
					if (!*timed_out_)
					{
						std::lock_guard<std::mutex> _(m_);
						fail.emplace_back(obj);
						errors.emplace_back(std::make_pair(obj, error::UnknownError(fmt::format("Unknown Error ({}:{})", FILENAME, __LINE__))));
					}
				}
				if (timeout_enabled  && !*timed_out_)
				{
					cv.notify_one();
				}
			});
			{
				std::unique_lock<std::mutex> l(m);
				if (timeout_enabled && cv.wait_for(l, timeout) == std::cv_status::timeout)
				{
					while (timed_out == nullptr)
					{
						std::this_thread::sleep_for(std::chrono::milliseconds(50));
					}
					*timed_out = true;
					std::lock_guard<std::mutex> _(m_);
					fail.emplace_back(obj);
					errors.emplace_back(std::make_pair(obj, error::TimeoutError(fmt::format("MultiThreader timed out at {} seconds ({}:{})", timeout.count(), FILENAME, __LINE__))));
					TerminateThread(t.native_handle(), 1);
					t.detach();
				}
				else
				{
					t.join();
				}
			}
		}));
	}
	for (std::thread &thread : threads)
	{
		thread.join();
	}
	threads.clear();
}

symb::scanner::helper::MultiThreader::MultiThreader(const MultiThreader &other)
{
	if (this != &other)
	{
		success = other.success;
		fail = other.fail;
		errors = other.errors;
	}
}

symb::scanner::helper::MultiThreader::~MultiThreader()
{
	success.clear();
	fail.clear();
	errors.clear();
}