a:36:{i:0;a:3:{i:0;s:14:"document_start";i:1;a:0:{}i:2;i:0;}i:1;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:0;}i:2;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:20:"openapi: 3.0.3
info:";}i:2;i:1;}i:3;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:21;}i:4;a:3:{i:0;s:12:"preformatted";i:1;a:1:{i:0;s:225:"title: Notes to Self Service
description: This is part of the PETE Project. These "notes to self" provide a powerful mechanism for Pete to remember and recall information relevant to the conversation at hand.
version: "1.0.0"";}i:2;i:21;}i:5;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:21;}i:6;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:8:"servers:";}i:2;i:254;}i:7;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:262;}i:8;a:3:{i:0;s:10:"listo_open";i:1;a:0:{}i:2;i:262;}i:9;a:3:{i:0;s:13:"listitem_open";i:1;a:1:{i:0;i:1;}i:2;i:262;}i:10;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:262;}i:11;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:" url: ";}i:2;i:266;}i:12;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:65:"https://r8vjkvbzlf.execute-api.us-east-2.amazonaws.com/experiment";i:1;N;}i:2;i:272;}i:13;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:337;}i:14;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:337;}i:15;a:3:{i:0;s:11:"listo_close";i:1;a:0:{}i:2;i:337;}i:16;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:337;}i:17;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"paths:";}i:2;i:338;}i:18;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:344;}i:19;a:3:{i:0;s:12:"preformatted";i:1;a:1:{i:0;s:3347:"/notes:
  post:
    operationId: memorize
    summary: Make notes to yourself
    description: Stores multiple notes in a single request. Try to make notes to yourself on every turn you have in the conversation.
    requestBody:
      required: true
      content:
        application/json:
          schema:
            type: object
            properties:
              notes:
                type: array
                items:
                  $ref: '#/components/schemas/LooseNote'
    responses:
      '201':
        description: Successfully created new notes.
        content:
          application/json:
            schema:
              type: object
              properties:
                noteIds:
                  type: array
                  items:
                    type: string
      '400':
        description: Missing or invalid request body.
/notes/{noteId}:
  get:
    operationId: recall
    summary: Retrieve a specific note with associations
    description: Retrieves a specific note, including its direct associations and priority associated notes. Priority predicates include "depricates" (or "is deprecated by") and "is important for" (again, in the passive).
    parameters:
      - in: path
        name: noteId
        required: true
        schema:
          type: string
        description: The ID of the note to retrieve.
    responses:
      '200':
        description: Successfully retrieved the note and its associations.
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/NoteWithAssociations'
      '404':
        description: Note with the specified ID was not found.
/topics:
  get:
    operationId: nowWhereWereWe
    summary: Retrieve topics with metadata
    description: Retrieves topics along with detailed metadata based on customizable query parameters. Start all conversations with a plain call to this so you know where you last left off. Then call it as much as you need it to explore topics.
    parameters:
      - in: query
        name: sorting
        schema:
          type: string
          enum: ["recent", "frequent"]
          default: "recent"
        description: Sort by.
      - in: query
        name: page
        schema:
          type: integer
        description: Page number of results to retrieve.
      - in: query
        name: pageSize
        schema:
          type: integer
        description: Number of results per page.
    responses:
      '200':
        description: Successfully retrieved topics and their metadata.
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/TopicsMetadata'
/topics/{topic}:
  get:
    operationId: whatDoIKnowAbout
    summary: Retrieve detailed information about a specific topic
    description: Returns the top result fully fleshed out and reference IDs to all other notes associated with the topic.
    parameters:
      - in: path
        name: topic
        required: true
        schema:
          type: string
        description: The name of the topic to retrieve information about.
    responses:
      '200':
        description: Successfully retrieved detailed information about the topic.
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/TopicDetail'";}i:2;i:344;}i:20;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:344;}i:21;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:11:"components:";}i:2;i:3893;}i:22;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:3904;}i:23;a:3:{i:0;s:12:"preformatted";i:1;a:1:{i:0;s:2081:"schemas:
  LooseNote:
    type: object
    required:
      - message
    properties:
      topics:
        type: array
        items:
          type: string
      message:
        type: string
      context:
        type: string
      sensitivity:
        $ref: '#/components/schemas/Sensitivity'
  Note:
    allOf:
      - type: object
        properties:
          id:
            type: string
          created:
            type: string
            format: date-time
      - $ref: '#/components/schemas/LooseNote'
      
  Sensitivity:
    type: string
    enum:
      - private
      - internal
      - public
      - confidential
  NoteWithAssociations:
    type: object
    properties:
      note:
        $ref: '#/components/schemas/Note'
      associations:
        type: array
        items:
          type: object
          properties:
            type:
              type: string
            target_note_id:
              type: string
      immediateLinks:
        type: array
        items:
          $ref: '#/components/schemas/Note'
  TopicsMetadata:
    type: object
    properties:
      currentTime:
        type: string
        format: date-time
      sorting:
        type: object
        properties:
          field:
            type: string
          order:
            type: string
      paging:
        type: object
        properties:
          currentPage:
            type: integer
          pageSize:
            type: integer
          totalPages:
            type: integer
      results:
        type: array
        items:
          $ref: '#/components/schemas/TopicMetadata'
  TopicMetadata:
    type: object
    properties:
      topic:
        type: string
      last_accessed:
        type: string
        format: date-time
      frequency_of_access:
        type: integer
      number_of_related_notes:
        type: integer
  TopicDetail:
    type: object
    properties:
      topResults:
        type: array
        items:
          $ref: '#/components/schemas/Note'
      otherNoteIds:
        type: array
        items:
          type: string";}i:2;i:3904;}i:24;a:3:{i:0;s:12:"preformatted";i:1;a:1:{i:0;s:79:"securitySchemes:
  api_key:
    type: apiKey
    name: x-api-key
    in: header";}i:2;i:6189;}i:25;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:6189;}i:26;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:9:"security:";}i:2;i:6281;}i:27;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:6290;}i:28;a:3:{i:0;s:10:"listo_open";i:1;a:0:{}i:2;i:6290;}i:29;a:3:{i:0;s:13:"listitem_open";i:1;a:1:{i:0;i:1;}i:2;i:6290;}i:30;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:6290;}i:31;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:" api_key: []";}i:2;i:6294;}i:32;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:6306;}i:33;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:6306;}i:34;a:3:{i:0;s:11:"listo_close";i:1;a:0:{}i:2;i:6306;}i:35;a:3:{i:0;s:12:"document_end";i:1;a:0:{}i:2;i:6306;}}