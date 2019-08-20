__kernel void blurPointBindKernel(
	__global float* finalPos,
	__global const float* initialPos,
	__global const float* targetPos,
	const uint positionCount
) {
	unsigned int positionId = get_global_id(0);
	if (positionId >= positionCount) return;
	unsigned int positionOffset = positionId * 3;
	float wpe = weights[positionId] * envelope;

	finalPos[positionOffset  ] = ((targetPos[positionOffset  ] - initialPos[positionOffset  ]) * wpe) + initialPos[positionOffset  ];
	finalPos[positionOffset+1] = ((targetPos[positionOffset+1] - initialPos[positionOffset+1]) * wpe) + initialPos[positionOffset+1];
	finalPos[positionOffset+2] = ((targetPos[positionOffset+2] - initialPos[positionOffset+2]) * wpe) + initialPos[positionOffset+2];

}

