/*
  ==============================================================================

    AmbixLoader.h
    Created: 5 Feb 2022 4:59:38pm
    Author:  sedur

  ==============================================================================
*/

#pragma once
#include "../JuceLibraryCode/JuceHeader.h"

class AmbixLoader
{
public:
	AmbixLoader(const juce::File& file);

	void getSourcePositions(std::vector<float>& azi, std::vector<float>& ele);
	void getDecodeMatrix(std::vector<float>& decodeMatrix);

	int getAmbiOrder();
	int getNumAmbiChans();
	int getNumLsChans();

	int getNumHrirs();
	void getHrir(int index, juce::AudioBuffer<float>& hrirs);

private:
	void parseFile(const juce::File& file);
	void parseDirection(const juce::String& filename);

	int m_order;
	int m_numAmbiChans;
	int m_numLsChans;

	std::vector<juce::AudioBuffer<float>> m_hrirs;
	std::vector<float> m_azi;
	std::vector<float> m_ele;
	std::vector<float> m_decodeMatrix;
};
