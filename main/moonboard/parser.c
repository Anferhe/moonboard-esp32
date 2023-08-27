/*
 * parser.c
 *
 *  Created on: Aug 27, 2023
 *      Author: yaroslav
 */

#include "parser.h"

void resetParser(Parser *parser)
{
	parser->startHoldsLen = 0;
	parser->moveHoldsLen= 0;
	parser->finishHoldsLen = 0;
	parser->sequenceLen = 0;

	parser->state = PARSER_START;
}

//! @brief Handle messages from app. Find start and finish
ParserState processSequence(Parser *parser, const void *data, size_t dataLen)
{
	static const unsigned char startSeq[2] = {"l#"};
	static const unsigned char endSeq = '#';

	static const size_t startSeqLen = sizeof(startSeq);
	static const size_t endSeqLen = sizeof(endSeq);

	if (!dataLen) {
		resetParser(parser);
		return PARSER_ERROR;
	}

	const uint8_t *buffer = (const uint8_t *)(data);
	size_t bufferLen = dataLen;


	switch (parser->state) {
		case PARSER_DONE:
			resetParser(parser);
			//! Fallthrough
		case PARSER_START:
			if (memcmp(startSeq, buffer, startSeqLen) == 0) {
				buffer += startSeqLen;
				bufferLen -= startSeqLen;
				parser->state = PARSER_DATA;
			} else {
				break;
			}
			//! Fallthrough
		case PARSER_DATA:
			if (endSeq == buffer[bufferLen - 1]) {
				bufferLen -= endSeqLen;
				parser->state = PARSER_DONE;
			}

			memcpy(&parser->sequence[parser->sequenceLen], buffer, bufferLen);
			parser->sequenceLen += bufferLen;
			break;
		default:
			break;
	}

	return parser->state;
}

#include "esp_log.h"

//! @brief Fill startHolds, moveHolds and finishHolds of Parser
bool parseSequence(Parser *parser)
{
	static const unsigned char startSymbol = 'S';
	static const unsigned char moveSymbol = 'P';
	static const unsigned char finishSymbol = 'E';
	static const unsigned char delimeterSymbol = ',';

	static char temp[10];

	char *tempPtr = &temp[0];

	//! CAUTION Using pointers we can change values out of arrays ranges
	//! Make sure that buffers can handle holds sequences
	uint8_t *startPtr = &parser->startHolds[0];
	uint8_t *movePtr = &parser->moveHolds[0];
	uint8_t *finishPtr = &parser->finishHolds[0];

	uint8_t **nextHoldPtr = &startPtr;
	size_t *nextCounterPtr = &parser->startHoldsLen;

	for (size_t i = 0; i < parser->sequenceLen; ++i) {
		switch (parser->sequence[i]) {
			case startSymbol:
				nextHoldPtr = &startPtr;
				nextCounterPtr = &parser->startHoldsLen;
				break;
			case moveSymbol:
				nextHoldPtr = &movePtr;
				nextCounterPtr = &parser->moveHoldsLen;
				break;
			case finishSymbol:
				nextHoldPtr = &finishPtr;
				nextCounterPtr = &parser->finishHoldsLen;
				break;
			case delimeterSymbol:
				*tempPtr = (char)0;  //! Add null character
				*(*nextHoldPtr)++ = atoi(temp);
				*nextCounterPtr += 1;
				tempPtr = &temp[0];
				break;
			default:
				*tempPtr++ = (char)parser->sequence[i];
				break;
		}
	}

	//! Add last hold
	*nextCounterPtr += 1;
	*tempPtr = (char)0;
	**nextHoldPtr = atoi(temp);

	return false;
}
