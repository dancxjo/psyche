a:26:{i:0;a:3:{i:0;s:14:"document_start";i:1;a:0:{}i:2;i:0;}i:1;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:0;}i:2;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:9:"openapi: ";}i:2;i:1;}i:3;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:10;}i:4;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"3.0.1";}i:2;i:11;}i:5;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:16;}i:6;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"
info:";}i:2;i:17;}i:7;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:23;}i:8;a:3:{i:0;s:12:"preformatted";i:1;a:1:{i:0;s:241:"title: "Notes to Self Service"
description: "This is part of the PETE Project. These \"notes to self\" provide\
  \ a powerful mechanism for Pete to remember and recall information relevant to\
  \ the conversation at hand."
version: "1.0.0"";}i:2;i:23;}i:9;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:23;}i:10;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:16:"servers:
- url: ";}i:2;i:276;}i:11;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:292;}i:12;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:55:"https://r8vjkvbzlf.execute-api.us-east-2.amazonaws.com/";i:1;N;}i:2;i:293;}i:13;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:10:"{basePath}";}i:2;i:348;}i:14;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:358;}i:15;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:359;}i:16;a:3:{i:0;s:12:"preformatted";i:1;a:1:{i:0;s:48:"variables:
  basePath:
    default: "experiment"";}i:2;i:359;}i:17;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:359;}i:18;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"paths:";}i:2;i:415;}i:19;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:421;}i:20;a:3:{i:0;s:12:"preformatted";i:1;a:1:{i:0;s:2955:"/topics:
  get:
    summary: "Retrieve topics with metadata"
    description: "Retrieves topics along with detailed metadata based on customizable\
      \ query parameters."
    operationId: "nowWhereWereWe"
    parameters:
    - name: "pageSize"
      in: "query"
      description: "Number of results per page."
      schema:
        type: "string"
    - name: "sorting"
      in: "query"
      description: "Sorting order and field."
      schema:
        type: "string"
    - name: "page"
      in: "query"
      description: "Page number of results to retrieve."
      schema:
        type: "string"
    responses:
      "200":
        description: "Successfully retrieved topics and their metadata."
        content:
          application/json:
            schema:
              $ref: "#/components/schemas/TopicsMetadata"
    security:
    - api_key: []
/notes/{noteId}:
  get:
    summary: "Retrieve a specific note with associations"
    description: "Retrieves a specific note, including its direct associations and\
      \ priority associated notes."
    operationId: "recallNote"
    parameters:
    - name: "noteId"
      in: "path"
      description: "The ID of the note to retrieve."
      required: true
      schema:
        type: "string"
    responses:
      "404":
        description: "Note with the specified ID was not found."
        content: {}
      "200":
        description: "Successfully retrieved the note and its associations."
        content:
          application/json:
            schema:
              $ref: "#/components/schemas/NoteWithAssociations"
    security:
    - api_key: []
/notes:
  post:
    summary: "Make notes to yourself"
    description: "Stores multiple notes in a single request."
    operationId: "memorizeNotes"
    requestBody:
      content:
        application/json:
          schema:
            $ref: "#/components/schemas/MODEL57b1df"
      required: true
    responses:
      "400":
        description: "Missing or invalid request body."
        content: {}
      "201":
        description: "Successfully created new notes."
        content:
          application/json:
            schema:
              $ref: "#/components/schemas/MODEL6420f6"
    security:
    - api_key: []
/topics/{topic}:
  get:
    summary: "Retrieve detailed information about a specific topic"
    description: "Returns the top result fully fleshed out and reference IDs to\
      \ all other notes associated with the topic."
    operationId: "whatDoIKnowAbout"
    parameters:
    - name: "topic"
      in: "path"
      description: "The name of the topic to retrieve information about."
      required: true
      schema:
        type: "string"
    responses:
      "200":
        description: "Successfully retrieved detailed information about the topic."
        content:
          application/json:
            schema:
              $ref: "#/components/schemas/TopicDetail"
    security:
    - api_key: []";}i:2;i:421;}i:21;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:421;}i:22;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:11:"components:";}i:2;i:3580;}i:23;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:3591;}i:24;a:3:{i:0;s:12:"preformatted";i:1;a:1:{i:0;s:2461:"schemas:
  MODEL6420f6:
    type: "object"
    properties:
      noteIds:
        type: "array"
        items:
          type: "string"
  Sensitivity:
    type: "string"
    enum:
    - "private"
    - "internal"
    - "public"
    - "confidential"
  TopicDetail:
    type: "object"
    properties:
      topResult:
        $ref: "#/components/schemas/Note"
      otherNoteIds:
        type: "array"
        items:
          type: "string"
  Note:
    required:
    - "message"
    type: "object"
    properties:
      topics:
        type: "array"
        items:
          type: "string"
      message:
        type: "string"
      context:
        type: "string"
      sensitivity:
        $ref: "#/components/schemas/Sensitivity"
  TopicsMetadata:
    type: "object"
    properties:
      currentTime:
        type: "string"
        format: "date-time"
      sorting:
        type: "object"
        properties:
          field:
            type: "string"
          order:
            type: "string"
      paging:
        type: "object"
        properties:
          currentPage:
            type: "integer"
            format: "int32"
          pageSize:
            type: "integer"
            format: "int32"
          totalPages:
            type: "integer"
            format: "int32"
      results:
        type: "array"
        items:
          $ref: "#/components/schemas/TopicMetadata"
  MODEL57b1df:
    type: "object"
    properties:
      notes:
        type: "array"
        items:
          $ref: "#/components/schemas/Note"
  NoteWithAssociations:
    type: "object"
    properties:
      _id:
        type: "string"
      topics:
        type: "array"
        items:
          type: "string"
      message:
        type: "string"
      context:
        type: "string"
      sensitivity:
        $ref: "#/components/schemas/Sensitivity"
      associations:
        type: "array"
        items:
          type: "object"
          properties:
            type:
              type: "string"
            target_note_id:
              type: "string"
  TopicMetadata:
    type: "object"
    properties:
      topic:
        type: "string"
      last_accessed:
        type: "string"
        format: "date-time"
      frequency_of_access:
        type: "integer"
        format: "int32"
      number_of_related_notes:
        type: "integer"
        format: "int32"
securitySchemes:
  api_key:
    type: "apiKey"
    name: "x-api-key"
    in: "header"";}i:2;i:3591;}i:25;a:3:{i:0;s:12:"document_end";i:1;a:0:{}i:2;i:3591;}}