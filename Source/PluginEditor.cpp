/*
  ==============================================================================

    This file contains the basic framework code for a JUCE plugin editor.

  ==============================================================================
*/

#include "PluginProcessor.h"
#include "PluginEditor.h"

//==============================================================================
NewProjectAudioProcessorEditor::NewProjectAudioProcessorEditor (NewProjectAudioProcessor& p)
    : AudioProcessorEditor (&p), audioProcessor (p)
{
    // Make sure that before the constructor has finished, you've set the
    // editor's size to whatever you need it to be.
	setSize(400, 200);

	addAndMakeVisible(&loadButton);
	loadButton.setButtonText("Load Ambix File");
	loadButton.onClick = [this] { loadButtonClicked(); };

	//panSlider.setSliderStyle(juce::Slider::LinearBarVertical);
	//panSlider.setRange(-180.f, 180.f, 0.1f); // TODO: Make log
	//panSlider.setTextBoxStyle(juce::Slider::TextBoxBelow, true, 20, 20);
	//panSlider.setPopupDisplayEnabled(true, true, this);
	//panSlider.setTextValueSuffix("Pan (x)");
	//panSlider.setValue(0.f);

	//tiltSlider.setSliderStyle(juce::Slider::LinearBarVertical);
	//tiltSlider.setRange(-180.f, 180.f, 0.1f); // TODO: Make log
	//tiltSlider.setTextBoxStyle(juce::Slider::TextBoxBelow, true, 20, 20);
	//tiltSlider.setPopupDisplayEnabled(true, true, this);
	//tiltSlider.setTextValueSuffix("Tilt (y)");
	//tiltSlider.setValue(0.f);

	//yawSlider.setSliderStyle(juce::Slider::LinearBarVertical);
	//yawSlider.setRange(-180.f, 180.f, 0.1f); // TODO: Make log
	//yawSlider.setTextBoxStyle(juce::Slider::TextBoxBelow, true, 20, 20);
	//yawSlider.setPopupDisplayEnabled(true, true, this);
	//yawSlider.setTextValueSuffix("Yaw (z)");
	//yawSlider.setValue(0.f);

	//addAndMakeVisible(panSlider);
	//addAndMakeVisible(tiltSlider);
	//addAndMakeVisible(yawSlider);

	//panSlider.addListener(this);
	//tiltSlider.addListener(this);
	//yawSlider.addListener(this);
}

NewProjectAudioProcessorEditor::~NewProjectAudioProcessorEditor()
{
}

//==============================================================================
void NewProjectAudioProcessorEditor::paint (juce::Graphics& g)
{
    // (Our component is opaque, so we must completely fill the background with a solid colour)
    g.fillAll (getLookAndFeel().findColour (juce::ResizableWindow::backgroundColourId));

	g.setColour(juce::Colours::black);
	g.setFont(15.0f);
	g.drawFittedText("Binaural Renderer", getLocalBounds(), juce::Justification::centred, 1);
}

void NewProjectAudioProcessorEditor::resized()
{
    // This is generally where you'll want to lay out the positions of any
    // subcomponents in your editor..
	loadButton.setBounds(10, 10, getWidth() - 20, 20);
	//panSlider.setBounds(140, 30, 40, getHeight() - 60);
	//tiltSlider.setBounds(240, 30, 40, getHeight() - 60);
	//yawSlider.setBounds(340, 30, 40, getHeight() - 60);
}

//void NewProjectAudioProcessorEditor::sliderValueChanged(juce::Slider* slider)
//{
//	if (slider == &panSlider)
//	{
//		audioProcessor.setPan(slider->getValue());
//	}
//	if (slider == &tiltSlider)
//	{
//		audioProcessor.setTilt(slider->getValue());
//	}
//	if (slider == &yawSlider)
//	{
//		audioProcessor.setYaw(slider->getValue());
//	}
//
//}

void NewProjectAudioProcessorEditor::loadButtonClicked()
{
	audioProcessor.enableRenderer(false);
	chooser = std::make_unique<juce::FileChooser>("Select an ambix config file",
		juce::File{},
		"*.config");                     // [7]
	auto chooserFlags = juce::FileBrowserComponent::openMode
		| juce::FileBrowserComponent::canSelectFiles;

	chooser->launchAsync(chooserFlags, [this](const juce::FileChooser& fc)           // [8]
	{
		auto file = fc.getResult();

		if (file != juce::File{})                                                      // [9]
		{
			audioProcessor.loadAmbix(file);
			audioProcessor.triggerHRIRUpload();
			audioProcessor.enableRenderer(true);
			//audioProcessor.enableRotation(true);
		}
	});

}