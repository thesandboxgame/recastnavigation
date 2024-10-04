#include "BuildContext.h"

BuildContext::BuildContext(TimeVal* accTimeUSec) : m_accTimeUSec(accTimeUSec)
{
	resetTimers();
}

// Virtual functions for custom implementations.
void BuildContext::doResetLog()
{
}

void BuildContext::doLog(const rcLogCategory category, const char* msg, const int len)
{
}

void BuildContext::doResetTimers()
{
	for (int i = 0; i < RC_MAX_TIMERS; ++i)
		m_accTime[i] = -1;
}

void BuildContext::doStartTimer(const rcTimerLabel label)
{
	m_startTime[label] = getPerfTime();
}

void BuildContext::doStopTimer(const rcTimerLabel label)
{
	const TimeVal endTime = getPerfTime();
	const TimeVal deltaTime = endTime - m_startTime[label];
	if (m_accTime[label] == -1)
		m_accTime[label] = deltaTime;
	else
		m_accTime[label] += deltaTime;
}

int BuildContext::doGetAccumulatedTime(const rcTimerLabel label) const
{
	return getPerfTimeUsec(m_accTime[label]);
}

void BuildContext::dumpLog(const char* format, ...)
{
}

int BuildContext::getLogCount() const
{
	return 0;
}

const char* BuildContext::getLogText(const int i) const
{
	return nullptr;
}

void BuildContext::computeAllTimings()
{
	for (int i = 0; i < RC_MAX_TIMERS; ++i)
	{
		if (m_accTime[i] >= 0)
		{
			m_accTimeUSec[i] = doGetAccumulatedTime((rcTimerLabel)i);
		}
		else
		{
			m_accTimeUSec[i] = 0;
		}
	}
}
