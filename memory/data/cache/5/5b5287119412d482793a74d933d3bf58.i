a:26:{i:0;a:3:{i:0;s:14:"document_start";i:1;a:0:{}i:2;i:0;}i:1;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:0;}i:2;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:9:"openapi: ";}i:2;i:1;}i:3;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:10;}i:4;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"3.0.1";}i:2;i:11;}i:5;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:16;}i:6;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"
info:";}i:2;i:17;}i:7;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:23;}i:8;a:3:{i:0;s:12:"preformatted";i:1;a:1:{i:0;s:50:"title: "ALIVE api"
version: "2024-03-21T20:20:34Z"";}i:2;i:23;}i:9;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:23;}i:10;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:16:"servers:
- url: ";}i:2;i:79;}i:11;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:95;}i:12;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:55:"https://0w1mc3e4gh.execute-api.us-east-2.amazonaws.com/";i:1;N;}i:2;i:96;}i:13;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:10:"{basePath}";}i:2;i:151;}i:14;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:161;}i:15;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:162;}i:16;a:3:{i:0;s:12:"preformatted";i:1;a:1:{i:0;s:45:"variables:
  basePath:
    default: "default"";}i:2;i:162;}i:17;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:162;}i:18;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"paths:";}i:2;i:215;}i:19;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:221;}i:20;a:3:{i:0;s:12:"preformatted";i:1;a:1:{i:0;s:501:"/log:
  post:
    description: "Record summaries of your daily actions, thoughts, reflections,\
      \ etc."
    operationId: "LogDiaryEntry"
    requestBody:
      content:
        application/json:
          schema:
            $ref: "#/components/schemas/DiaryEntry"
      required: true
    responses:
      "200":
        description: "200 response"
        content:
          application/json:
            schema:
              $ref: "#/components/schemas/Empty"
    security:
    - api_key: []";}i:2;i:221;}i:21;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:221;}i:22;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:11:"components:";}i:2;i:764;}i:23;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:775;}i:24;a:3:{i:0;s:12:"preformatted";i:1;a:1:{i:0;s:424:"schemas:
  Empty:
    title: "Empty Schema"
    type: "object"
  DiaryEntry:
    title: "MessageModel"
    required:
    - "message"
    type: "object"
    properties:
      message:
        type: "string"
      dateTime:
        type: "string"
        description: "Optional date and time in ISO 8601 format."
        format: "date-time"
securitySchemes:
  api_key:
    type: "apiKey"
    name: "x-api-key"
    in: "header"";}i:2;i:775;}i:25;a:3:{i:0;s:12:"document_end";i:1;a:0:{}i:2;i:775;}}