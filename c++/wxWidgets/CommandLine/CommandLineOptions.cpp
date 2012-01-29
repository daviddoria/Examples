//
// Copyright 2010, Darren Lafreniere
// <http://www.lafarren.com/image-completer/>
//
// This file is part of lafarren.com's Image Completer.
//
// Image Completer is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Image Completer is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Image Completer, named License.txt. If not, see
// <http://www.gnu.org/licenses/>.
//

#include "CommandLineOptions.h"

#include <wx/cmdline.h>

//
// Constants
//
static const wxString CMD_IMAGE_INPUT                               = wxString::FromAscii("ii");
static const wxString CMD_IMAGE_MASK                                = wxString::FromAscii("im");
static const wxString CMD_IMAGE_OUTPUT                              = wxString::FromAscii("io");

static const wxString CMD_SETTINGS_SHOW                             = wxString::FromAscii("ss");
static const wxString CMD_SETTINGS_LOW_RESOLUTION_PASSES_MAX        = wxString::FromAscii("sp");
static const wxString CMD_SETTINGS_DEBUG_LOW_RESOLUTION_PASSES      = wxString::FromAscii("sd");
static const wxString CMD_SETTINGS_NUM_ITERATIONS                   = wxString::FromAscii("si");
static const wxString CMD_SETTINGS_LATTICE_GAP_WIDTH                = wxString::FromAscii("sw");
static const wxString CMD_SETTINGS_LATTICE_GAP_HEIGHT               = wxString::FromAscii("sh");
static const wxString CMD_SETTINGS_POST_PRUNE_PATCHES_MIN           = wxString::FromAscii("smn");
static const wxString CMD_SETTINGS_POST_PRUNE_PATCHES_MAX           = wxString::FromAscii("smx");
static const wxString CMD_SETTINGS_COMPOSITOR_PATCH_TYPE            = wxString::FromAscii("sct");
static const wxString CMD_SETTINGS_COMPOSITOR_PATCH_BLENDER         = wxString::FromAscii("scb");

#if ENABLE_PATCHES_INPUT_OUTPUT
static const wxString CMD_PATCHES_INPUT                             = wxString::FromAscii("pi");
static const wxString CMD_PATCHES_OUTPUT                            = wxString::FromAscii("po");
#endif // ENABLE_PATCHES_INPUT_OUTPUT

// Lets our wxCmdLineEntryDesc descriptions reference the char* buffer of an
// inline constructed wxString, without worrying about the scope of that
// wxString. Also provides some pre and post formatting for consistency.
class Desc
{
public:
	Desc(const wxString& desc)
	{
		wxASSERT(m_staticStringsSize < MAX);
		m_index = m_staticStringsSize++;
		m_staticStrings[m_index] = wxString::Format(wxT("\n%s%s\n"), Indent(), desc.c_str());
	}

	operator const char*() const
	{
		return m_staticStrings[m_index].mb_str();
	}

	inline static const char* Indent() { return "   "; }


private:
	static const int MAX = 256;
	static wxString m_staticStrings[MAX];
	static int m_staticStringsSize;
	int m_index;
};
wxString Desc::m_staticStrings[MAX];
int Desc::m_staticStringsSize;

//
// CommandLineOptions::ValueFinder and partial specializations.
//
template<typename T>
void CommandLineOptions::ValueFinder<T>::Find(const wxCmdLineParser& parser, const wxString& name)
{
	wasFound = parser.Found(name, &value);
}

template<>
void CommandLineOptions::ValueFinder<CommandLineOptions::LowResolutionPassesMax>::Find(const wxCmdLineParser& parser, const wxString& name)
{
	wxString stringValue;
	if (parser.Found(name, &stringValue))
	{

	}
}


//
// CommandLineOptions
//
CommandLineOptions::CommandLineOptions(int argc, char** argv)
	: m_isValid(false)
	, m_shouldShowSettings(false)
	, m_shouldRunImageCompletion(false)
	, m_debugLowResolutionPasses(false)
{
	// Create lists of PriorityBp::CompositorPatchType and
	// PriorityBp::CompositorPatchBlender options
	wxString compositorPatchTypeOptions;
	wxString compositorPatchBlenderOptions;

    const wxCmdLineEntryDesc CMD_LINE_DESC[] =
	{
		{ wxCMD_LINE_OPTION, CMD_IMAGE_INPUT, wxT("image-input"), wxString::FromAscii(Desc(wxString::FromAscii("The input image file path."))), wxCMD_LINE_VAL_STRING, wxCMD_LINE_OPTION_MANDATORY },
		{ wxCMD_LINE_OPTION, CMD_IMAGE_MASK, wxT("image-mask"), wxString::FromAscii(Desc(wxString::FromAscii("The mask image file path."))), wxCMD_LINE_VAL_STRING, wxCMD_LINE_PARAM_OPTIONAL },
		{ wxCMD_LINE_OPTION, CMD_IMAGE_OUTPUT, wxT("image-output"), wxString::FromAscii(Desc(wxString::FromAscii("The output image file path."))), wxCMD_LINE_VAL_STRING, wxCMD_LINE_PARAM_OPTIONAL },

		{ wxCMD_LINE_NONE }
	};
	wxCmdLineParser parser(argc, argv);
	parser.SetLogo(wxString::FromAscii("\nlafarren.com\nImage Completion Using Efficient Belief Propagation\n"));
	parser.SetSwitchChars(wxString::FromAscii("-"));
	parser.SetDesc(CMD_LINE_DESC);
	if (parser.Parse() == 0)
	{
		parser.Found(CMD_IMAGE_INPUT, &m_inputImagePath);
		parser.Found(CMD_IMAGE_MASK, &m_maskImagePath);
		parser.Found(CMD_IMAGE_OUTPUT, &m_outputImagePath);

#if ENABLE_PATCHES_INPUT_OUTPUT
		parser.Found(CMD_PATCHES_INPUT, &m_inputPatchesPath);
		parser.Found(CMD_PATCHES_OUTPUT, &m_outputPatchesPath);
#endif // ENABLE_PATCHES_INPUT_OUTPUT

		m_shouldShowSettings = parser.Found(CMD_SETTINGS_SHOW);

		// As of now, don't run image completion if the user wanted the
		// settings displayed. Maybe change this later.
		m_shouldRunImageCompletion = !m_shouldShowSettings;

		m_isValid = true;

		// Invalidate if something is missing.
		{
			wxString errorMessage;

			// wxparser::Parse() should've failed if the input image
			// wasn't present, since it's set to wxCMD_LINE_OPTION_MANDATORY.
			wxASSERT(HasInputImagePath());

			if (m_shouldShowSettings)
			{
				// Nothing besides the input image is needed to display the settings.
			}

			if (m_shouldRunImageCompletion)
			{
				// Running image completion requires mask and output images.
				if (!HasMaskImagePath() && !HasOutputImagePath())
				{
				}
				else if (!HasMaskImagePath())
				{
				}
				else if (!HasOutputImagePath())
				{
				}
			}

			if (!m_isValid)
			{
				parser.Usage();

			}
			else
			{
				// These options are optional. If anything is invalid,
				// Priority::Settings::IsValid() will catch it later.
				m_debugLowResolutionPasses = parser.Found(CMD_SETTINGS_DEBUG_LOW_RESOLUTION_PASSES);
				m_lowResolutionPassesMax.Find(parser, CMD_SETTINGS_LOW_RESOLUTION_PASSES_MAX);
				m_numIterations.Find(parser, CMD_SETTINGS_NUM_ITERATIONS);
				m_latticeGapX.Find(parser, CMD_SETTINGS_LATTICE_GAP_WIDTH);
				m_latticeGapY.Find(parser, CMD_SETTINGS_LATTICE_GAP_HEIGHT);
				m_postPruneLabelsMin.Find(parser, CMD_SETTINGS_POST_PRUNE_PATCHES_MIN);
				m_postPruneLabelsMax.Find(parser, CMD_SETTINGS_POST_PRUNE_PATCHES_MAX);
			}
		}
	}
}
