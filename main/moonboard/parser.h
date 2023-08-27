/*
 * parser.h
 *
 *  Created on: Aug 27, 2023
 *      Author: yaroslav
 */

#ifndef MOONBOARD_PARSER_H_
#define MOONBOARD_PARSER_H_

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#define UNUSED_HOLD_VALUE  0xFF
#define TEMP_BUFFER_LEN 10

#define START_HOLDS_BUFFER_LEN 5
#define MOVE_HOLDS_BUFFER_LEN 20
#define FINISH_HOLDS_BUFFER_LEN 5
#define MAX_SEQUENCE_LEN 500

enum ParserState {
	PARSER_START,
	PARSER_DATA,
	PARSER_DONE,
	PARSER_ERROR
};

typedef enum ParserState ParserState;

struct Parser {
	ParserState state;

	uint8_t startHolds[START_HOLDS_BUFFER_LEN];
	uint8_t moveHolds[MOVE_HOLDS_BUFFER_LEN];
	uint8_t finishHolds[FINISH_HOLDS_BUFFER_LEN];
	uint8_t sequence[500];

	size_t startHoldsLen;
	size_t moveHoldsLen;
	size_t finishHoldsLen;
	size_t sequenceLen;
};

//! Set PARSER_DONE state to force parser reset
typedef struct Parser Parser;

void resetParser(Parser *parser);
ParserState processSequence(Parser *parser, const void *data, size_t dataLen);
bool parseSequence(Parser *parser);

#endif /* MOONBOARD_PARSER_H_ */
