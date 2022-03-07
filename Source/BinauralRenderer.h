/*
  ==============================================================================

    BinauralRenderer.h
    Created: 5 Feb 2022 4:59:07pm
    Author:  sedur

  ==============================================================================
*/

#pragma once
#include "../JuceLibraryCode/JuceHeader.h"
#include "WDL/convoengine.h"
#include "WDL/fft.h"
//#include "AmbisonicRotation.h"
#include "AmbixLoader.h"

class BinauralRenderer
	: public juce::ChangeBroadcaster
{
public:
	BinauralRenderer();

	void setOrder(const int order);
	int getOrder() const;

	void clearVirtualLoudspeakers();
	void setVirtualLoudspeakers(const std::vector<float>& azi, const std::vector<float>& ele, const int chans);
	void getVirtualLoudspeakers(std::vector<float>& azi, std::vector<float>& ele, int& chans);

	//void setHeadTrackingData(float yaw, float pitch, float roll);

	//float getRoll();
	//float getPitch();
	//float getYaw();

	bool isRendererEnabled();

	void enableRenderer(bool enable);
	void enableDualBand(bool enable);
	//void enableRotation(bool enable);

	void prepareToPlay(int samplesPerBlockExpected, double sampleRate);
	void processBlock(juce::AudioBuffer<float>& buffer);
	void releaseResources();

	void sendMsgToLogWindow(juce::String message);
	juce::String m_currentLogMessage;

	void clearHRIR();
	void addHRIR(const juce::AudioBuffer<float>& buffer);
	bool uploadHRIRsToEngine();

	void loadAmbixFile(const juce::File& ambixFile);

	class Listener
	{
	public:
		virtual ~Listener() = default;
		virtual void ambixFileLoaded(const juce::File& file) = 0;
	};

	void addListener(Listener* newListener);
	void removeListener(Listener* listener);

private:
	void setDecodingMatrix(std::vector<float>& decodeMatrix);
	void updateMatrices();

	bool convertHRIRToSHDHRIR();

	juce::CriticalSection m_procLock;

	juce::AudioBuffer<float> m_workingBuffer;
	juce::AudioBuffer<float> m_convBuffer;

	int m_order;
	int m_numAmbiChans;
	int m_numLsChans;
	int m_numHrirAdded;
	int m_blockSize;
	double m_sampleRate;

	//float m_yaw;
	//float m_pitch;
	//float m_roll;

	std::vector<juce::AudioBuffer<float>> m_hrirBuffers;
	std::vector<juce::AudioBuffer<float>> m_hrirShdBuffers;

	std::vector<float> m_basicDecodeMatrix;
	std::vector<float> m_basicDecodeTransposeMatrix;

	std::vector<float> m_azi;
	std::vector<float> m_ele;

	//DualBandFilter m_dbFilter;
	//AmbisonicRotation m_headTrackRotator;

	std::vector<std::unique_ptr<WDL_ConvolutionEngine>> m_convEngines;

	bool m_enableRenderer;
	bool m_enableDualBand;
	bool m_enableRotation;

	juce::ListenerList<Listener> listeners;

	void mat_trans(float* outmtx, float* inmtx, int rows, int cols)
	{
		int i, j;

		for (i = 0; i < rows; ++i)
		{
			for (j = 0; j < cols; ++j)
			{
				outmtx[(j * rows) + i] = inmtx[(i * cols) + j];
			}
		}
	}

	void mat_mult(float* out, const float* A, const float* B, int n, int m, int m2, int p)
	{
		int i, j, k;
		float s;

		for (i = 0; i < n; ++i)
		{
			for (j = 0; j < p; ++j)
			{
				s = 0.0f;

				for (k = 0; k < m; ++k)
					s += A[(i * m) + k] * B[(k * p) + j];

				out[(i * p) + j] = s;
			}
		}
	}

	float legendreP(const int n, const float x)
	{
		if (n == 0)
			return 1.0f;
		else if (n == 1)
			return x;
		else
			return ((((2 * (n - 1)) + 1) * x * legendreP(n - 1, x)) - ((n - 1) * legendreP(n - 2, x))) / n;
	}

	JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(BinauralRenderer)
};
