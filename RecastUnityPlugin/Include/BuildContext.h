#pragma once
#include "PerfTimer.h"
#include "Recast.h"

// Inspired by BuildContext in SampleInterfaces
/// A context to be able to gather the timings. The logs are not supported for the time being.
class BuildContext : public rcContext
{
public:
	BuildContext(TimeVal* m_accTimeUSec);

	/// Dumps the log to stdout.
	void dumpLog(const char* format, ...);
	/// Returns number of log messages.
	int getLogCount() const;
	/// Returns log message text.
	const char* getLogText(const int i) const;

	void computeAllTimings();

protected:
	/// Virtual functions for custom implementations.
	///@{
	virtual void doResetLog();
	virtual void doLog(const rcLogCategory category, const char* msg, const int len);
	virtual void doResetTimers();
	virtual void doStartTimer(const rcTimerLabel label);
	virtual void doStopTimer(const rcTimerLabel label);
	virtual int doGetAccumulatedTime(const rcTimerLabel label) const;
	///@}

private:
	TimeVal* m_accTimeUSec;
	TimeVal m_startTime[RC_MAX_TIMERS];
	TimeVal m_accTime[RC_MAX_TIMERS];
};
