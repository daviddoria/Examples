	// Generate a random image
	/*
	for(unsigned int r = 0; r < NumRows; r++)
{
		for(unsigned int c = 0; c < NumCols; c++)
{
			ImageType::IndexType pixelIndex;
			pixelIndex[0] = r;
			pixelIndex[1] = c;
			
			float RandVal = drand48();
			
			float Snapped;
			if(RandVal <= .5)
				Snapped = 0.0;
			else
				Snapped = 1.0;
			
			image->SetPixel(pixelIndex, Snapped);
}
}
	*/