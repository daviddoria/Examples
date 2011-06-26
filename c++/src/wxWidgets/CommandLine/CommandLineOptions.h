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

#ifndef COMMAND_LINE_OPTIONS_H
#define COMMAND_LINE_OPTIONS_H

#include "wx/string.h"

class wxCmdLineParser;

// Reading/writing a patches file is a development-only thing when doing
// compositor work. Enable this, and use -po patches.filename on the command
// line, and the next image completion will write its solved patches data to
// patches.filename. On the next run, use -pi patches.filename to skip the
// expensive process of image completion, and read the previously written
// patches data to send to the compositor.
#define ENABLE_PATCHES_INPUT_OUTPUT 1

// Pulls the command line options from the parser, validates them, and
// presents a strongly typed interface to their values.
class CommandLineOptions
{
public:
	CommandLineOptions(int argc, char** argv);

	inline bool IsValid() const { return m_isValid; }

	inline bool HasInputImagePath() const { return !m_inputImagePath.IsEmpty(); }
	inline bool HasMaskImagePath() const { return !m_maskImagePath.IsEmpty(); }
	inline bool HasOutputImagePath() const { return !m_outputImagePath.IsEmpty(); }

	inline const wxString& GetInputImagePath() const { return m_inputImagePath; }
	inline const wxString& GetMaskImagePath() const { return m_maskImagePath; }
	inline const wxString& GetOutputImagePath() const { return m_outputImagePath; }

#if ENABLE_PATCHES_INPUT_OUTPUT
	inline bool HasInputPatchesPath() const { return !m_inputPatchesPath.IsEmpty(); }
	inline bool HasOutputPatchesPath() const { return !m_outputPatchesPath.IsEmpty(); }

	inline const wxString& GetInputPatchesPath() const { return m_inputPatchesPath; }
	inline const wxString& GetOutputPatchesPath() const { return m_outputPatchesPath; }
#endif // ENABLE_PATCHES_INPUT_OUTPUT

	inline bool ShouldShowSettings() const { return m_shouldShowSettings; }
	inline bool ShouldRunImageCompletion() const { return m_shouldRunImageCompletion; }

	inline bool DebugLowResolutionPasses() const { return m_debugLowResolutionPasses; }

	inline bool HasLowResolutionPassesMax() const { return m_lowResolutionPassesMax.wasFound; }
	inline int GetLowResolutionPassesMax() const { return m_lowResolutionPassesMax.value; }

	inline bool HasNumIterations() const { return m_numIterations.wasFound; }
	inline int GetNumIterations() const { return m_numIterations.value; }

	inline bool HasLatticeGapX() const { return m_latticeGapX.wasFound; }
	inline int GetLatticeGapX() const { return m_latticeGapX.value; }

	inline bool HasLatticeGapY() const { return m_latticeGapY.wasFound; }
	inline int GetLatticeGapY() const { return m_latticeGapY.value; }

	inline bool HasPostPruneLabelsMin() const { return m_postPruneLabelsMin.wasFound; }
	inline int GetPostPruneLabelsMin() const { return m_postPruneLabelsMin.value; }

	inline bool HasPostPruneLabelsMax() const { return m_postPruneLabelsMax.wasFound; }
	inline int GetPostPruneLabelsMax() const { return m_postPruneLabelsMax.value; }


private:
	wxString m_inputImagePath;
	wxString m_maskImagePath;
	wxString m_outputImagePath;
#if ENABLE_PATCHES_INPUT_OUTPUT
	wxString m_inputPatchesPath;
	wxString m_outputPatchesPath;
#endif // ENABLE_PATCHES_INPUT_OUTPUT

	template<typename T>
	struct ValueFinder
	{
		bool wasFound;
		T value;

		ValueFinder() : wasFound(false) {}
		void Find(const wxCmdLineParser& parser, const wxString& name);
	};

	// Solely for specialization.
	typedef int LowResolutionPassesMax;

	// Some of these are longs instead of ints because of
	// wxCmdLineParser::Found(const wxString&, long*)
	ValueFinder<LowResolutionPassesMax> m_lowResolutionPassesMax;
	ValueFinder<long> m_numIterations;
	ValueFinder<long> m_latticeGapX;
	ValueFinder<long> m_latticeGapY;
	ValueFinder<long> m_postPruneLabelsMin;
	ValueFinder<long> m_postPruneLabelsMax;

	bool m_shouldShowSettings;
	bool m_shouldRunImageCompletion;
	bool m_debugLowResolutionPasses;
	bool m_isValid;
};

#endif // COMMAND_LINE_OPTIONS_H
