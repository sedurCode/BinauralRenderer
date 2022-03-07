/*
  ==============================================================================

    This file contains the basic framework code for a JUCE plugin processor.

  ==============================================================================
*/

#pragma once

#include <JuceHeader.h>
#include "BinauralRenderer.h"

//==============================================================================
/**
*/
class NewProjectAudioProcessor  : public juce::AudioProcessor
{
public:
    //==============================================================================
    NewProjectAudioProcessor();
    ~NewProjectAudioProcessor() override;

    //==============================================================================
    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override;

   #ifndef JucePlugin_PreferredChannelConfigurations
    bool isBusesLayoutSupported (const BusesLayout& layouts) const override;
   #endif

    void processBlock (juce::AudioBuffer<float>&, juce::MidiBuffer&) override;

    //==============================================================================
    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override;

    //==============================================================================
    const juce::String getName() const override;

    bool acceptsMidi() const override;
    bool producesMidi() const override;
    bool isMidiEffect() const override;
    double getTailLengthSeconds() const override;

    //==============================================================================
    int getNumPrograms() override;
    int getCurrentProgram() override;
    void setCurrentProgram (int index) override;
    const juce::String getProgramName (int index) override;
    void changeProgramName (int index, const juce::String& newName) override;

    //==============================================================================
    void getStateInformation (juce::MemoryBlock& destData) override;
    void setStateInformation (const void* data, int sizeInBytes) override;
	//void setGain(float newGain) { fTargetGain = newGain; };
	//void setPan(float newPan) { fTargetPan = newPan; };
	//void setTilt(float newTilt) { fTargetTilt = newTilt; };
	//void setYaw(float newYaw) { fTargetYaw = newYaw; };
    void loadAmbix(juce::File file) { renderer.loadAmbixFile(file); };
    void enableRenderer(bool enable) { renderer.enableRenderer(enable); };
	void enableDualBand(bool enable) {renderer.enableDualBand(enable);};
	//void enableRotation(bool enable) {renderer.enableRotation(enable);};
    void triggerHRIRUpload() { renderer.uploadHRIRsToEngine(); };
private:

    void updateParameters();

    BinauralRenderer renderer;
	//// Targets
	//float fTargetGain = 1.f;
	//float fTargetPan = 0.f;
	//float fTargetTilt = 0.f;
	//float fTargetYaw = 0.f;
	//// Current Values
	//float fCurrentGain = 0.f;
	//float fCurrentPan = 0.f;
	//float fCurrentTilt = 0.f;
	//float fCurrentYaw = 0.f;
	//// Steps
	//float fGainStep = 0.f;
	//float fPanStep = 0.f;
	//float fTiltStep = 0.f;
	//float fYawStep = 0.f;
	//// Constants
	//const float fMaxDeltaStep = 1.f / 480.f;
    // TODO headphone Compensation 
    //==============================================================================
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (NewProjectAudioProcessor)
};
