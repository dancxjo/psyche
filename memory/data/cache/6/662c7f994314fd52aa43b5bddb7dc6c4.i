a:2065:{i:0;a:3:{i:0;s:14:"document_start";i:1;a:0:{}i:2;i:0;}i:1;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:0;}i:2;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:11:"# [Brex's](";}i:2;i:1;}i:3;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:16:"https://brex.com";i:1;N;}i:2;i:12;}i:4;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:26:") Prompt Engineering Guide";}i:2;i:28;}i:5;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:54;}i:6;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:54;}i:7;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:377:"This guide was created by Brex for internal purposes. It's based on
lessons learned from researching and creating Large Language Model (LLM)
prompts for production use cases. It covers the history around LLMs as well as
strategies, guidelines, and safety recommendations for working with and
building programmatic systems on top of large language models, like [OpenAI's
GPT-4](";}i:2;i:56;}i:8;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:33:"https://openai.com/research/gpt-4";i:1;N;}i:2;i:433;}i:9;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:").";}i:2;i:466;}i:10;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:468;}i:11;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:468;}i:12;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:138:"The examples in this document were generated with a non-deterministic language
model and the same examples may give you different results.";}i:2;i:470;}i:13;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:608;}i:14;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:608;}i:15;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:181:"This is a living document. The state-of-the-art best practices and strategies
around LLMs are evolving rapidly every day. Discussion and suggestions for
improvements are encouraged.";}i:2;i:610;}i:16;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:791;}i:17;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:791;}i:18;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:93:"## Table of Contents
- [What is a Large Language Model?](#what-is-a-large-language-model-llm)";}i:2;i:793;}i:19;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:886;}i:20;a:3:{i:0;s:10:"listo_open";i:1;a:0:{}i:2;i:886;}i:21;a:3:{i:0;s:13:"listitem_open";i:1;a:2:{i:0;i:1;i:1;i:1;}i:2;i:886;}i:22;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:886;}i:23;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:144:" [A Brief, Incomplete, and Somewhat Incorrect History of Language Models](#a-brief-incomplete-and-somewhat-incorrect-history-of-language-models)";}i:2;i:890;}i:24;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:1034;}i:25;a:3:{i:0;s:10:"listo_open";i:1;a:0:{}i:2;i:1034;}i:26;a:3:{i:0;s:13:"listitem_open";i:1;a:1:{i:0;i:2;}i:2;i:1034;}i:27;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:1034;}i:28;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:27:" [Pre-2000’s](#pre-2000s)";}i:2;i:1040;}i:29;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:1067;}i:30;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:1067;}i:31;a:3:{i:0;s:13:"listitem_open";i:1;a:1:{i:0;i:2;}i:2;i:1067;}i:32;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:1067;}i:33;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:27:" [Mid-2000’s](#mid-2000s)";}i:2;i:1073;}i:34;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:1100;}i:35;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:1100;}i:36;a:3:{i:0;s:13:"listitem_open";i:1;a:1:{i:0;i:2;}i:2;i:1100;}i:37;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:1100;}i:38;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:31:" [Early-2010’s](#early-2010s)";}i:2;i:1106;}i:39;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:1137;}i:40;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:1137;}i:41;a:3:{i:0;s:13:"listitem_open";i:1;a:1:{i:0;i:2;}i:2;i:1137;}i:42;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:1137;}i:43;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:29:" [Late-2010’s](#late-2010s)";}i:2;i:1143;}i:44;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:1172;}i:45;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:1172;}i:46;a:3:{i:0;s:13:"listitem_open";i:1;a:1:{i:0;i:2;}i:2;i:1172;}i:47;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:1172;}i:48;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:19:" [2020’s](#2020s)";}i:2;i:1178;}i:49;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:1197;}i:50;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:1197;}i:51;a:3:{i:0;s:11:"listo_close";i:1;a:0:{}i:2;i:1197;}i:52;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:1197;}i:53;a:3:{i:0;s:11:"listo_close";i:1;a:0:{}i:2;i:1197;}i:54;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:1197;}i:55;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:40:"- [What is a prompt?](#what-is-a-prompt)";}i:2;i:1198;}i:56;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:1238;}i:57;a:3:{i:0;s:10:"listo_open";i:1;a:0:{}i:2;i:1238;}i:58;a:3:{i:0;s:13:"listitem_open";i:1;a:1:{i:0;i:1;}i:2;i:1238;}i:59;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:1238;}i:60;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:34:" [Hidden Prompts](#hidden-prompts)";}i:2;i:1242;}i:61;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:1276;}i:62;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:1276;}i:63;a:3:{i:0;s:13:"listitem_open";i:1;a:1:{i:0;i:1;}i:2;i:1276;}i:64;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:1276;}i:65;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:18:" [Tokens](#tokens)";}i:2;i:1280;}i:66;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:1298;}i:67;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:1298;}i:68;a:3:{i:0;s:13:"listitem_open";i:1;a:1:{i:0;i:1;}i:2;i:1298;}i:69;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:1298;}i:70;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:30:" [Token Limits](#token-limits)";}i:2;i:1302;}i:71;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:1332;}i:72;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:1332;}i:73;a:3:{i:0;s:13:"listitem_open";i:1;a:2:{i:0;i:1;i:1;i:1;}i:2;i:1332;}i:74;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:1332;}i:75;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:34:" [Prompt Hacking](#prompt-hacking)";}i:2;i:1336;}i:76;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:1370;}i:77;a:3:{i:0;s:10:"listo_open";i:1;a:0:{}i:2;i:1370;}i:78;a:3:{i:0;s:13:"listitem_open";i:1;a:1:{i:0;i:2;}i:2;i:1370;}i:79;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:1370;}i:80;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:26:" [Jailbreaks](#jailbreaks)";}i:2;i:1376;}i:81;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:1402;}i:82;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:1402;}i:83;a:3:{i:0;s:13:"listitem_open";i:1;a:1:{i:0;i:2;}i:2;i:1402;}i:84;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:1402;}i:85;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:16:" [Leaks](#leaks)";}i:2;i:1408;}i:86;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:1424;}i:87;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:1424;}i:88;a:3:{i:0;s:11:"listo_close";i:1;a:0:{}i:2;i:1424;}i:89;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:1424;}i:90;a:3:{i:0;s:11:"listo_close";i:1;a:0:{}i:2;i:1424;}i:91;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:1424;}i:92;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:74:"- [Why do we need prompt engineering?](#why-do-we-need-prompt-engineering)";}i:2;i:1425;}i:93;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:1499;}i:94;a:3:{i:0;s:10:"listo_open";i:1;a:0:{}i:2;i:1499;}i:95;a:3:{i:0;s:13:"listitem_open";i:1;a:2:{i:0;i:1;i:1;i:1;}i:2;i:1499;}i:96;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:1499;}i:97;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:40:" [Give a Bot a Fish](#give-a-bot-a-fish)";}i:2;i:1503;}i:98;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:1543;}i:99;a:3:{i:0;s:10:"listo_open";i:1;a:0:{}i:2;i:1543;}i:100;a:3:{i:0;s:13:"listitem_open";i:1;a:1:{i:0;i:2;}i:2;i:1543;}i:101;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:1543;}i:102;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:36:" [Semantic Search](#semantic-search)";}i:2;i:1549;}i:103;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:1585;}i:104;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:1585;}i:105;a:3:{i:0;s:11:"listo_close";i:1;a:0:{}i:2;i:1585;}i:106;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:1585;}i:107;a:3:{i:0;s:13:"listitem_open";i:1;a:2:{i:0;i:1;i:1;i:1;}i:2;i:1585;}i:108;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:1585;}i:109;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:44:" [Teach a Bot to Fish](#teach-a-bot-to-fish)";}i:2;i:1589;}i:110;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:1633;}i:111;a:3:{i:0;s:10:"listo_open";i:1;a:0:{}i:2;i:1633;}i:112;a:3:{i:0;s:13:"listitem_open";i:1;a:1:{i:0;i:2;}i:2;i:1633;}i:113;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:1633;}i:114;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:38:" [Command Grammars](#command-grammars)";}i:2;i:1639;}i:115;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:1677;}i:116;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:1677;}i:117;a:3:{i:0;s:13:"listitem_open";i:1;a:1:{i:0;i:2;}i:2;i:1677;}i:118;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:1677;}i:119;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:16:" [ReAct](#react)";}i:2;i:1683;}i:120;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:1699;}i:121;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:1699;}i:122;a:3:{i:0;s:13:"listitem_open";i:1;a:1:{i:0;i:2;}i:2;i:1699;}i:123;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:1699;}i:124;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:37:" [GPT-4 vs GPT-3.5](#gpt-4-vs-gpt-35)";}i:2;i:1705;}i:125;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:1742;}i:126;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:1742;}i:127;a:3:{i:0;s:11:"listo_close";i:1;a:0:{}i:2;i:1742;}i:128;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:1742;}i:129;a:3:{i:0;s:11:"listo_close";i:1;a:0:{}i:2;i:1742;}i:130;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:1742;}i:131;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:27:"- [Strategies](#strategies)";}i:2;i:1743;}i:132;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:1770;}i:133;a:3:{i:0;s:10:"listo_open";i:1;a:0:{}i:2;i:1770;}i:134;a:3:{i:0;s:13:"listitem_open";i:1;a:2:{i:0;i:1;i:1;i:1;}i:2;i:1770;}i:135;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:1770;}i:136;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:34:" [Embedding Data](#embedding-data)";}i:2;i:1774;}i:137;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:1808;}i:138;a:3:{i:0;s:10:"listo_open";i:1;a:0:{}i:2;i:1808;}i:139;a:3:{i:0;s:13:"listitem_open";i:1;a:1:{i:0;i:2;}i:2;i:1808;}i:140;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:1808;}i:141;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:30:" [Simple Lists](#simple-lists)";}i:2;i:1814;}i:142;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:1844;}i:143;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:1844;}i:144;a:3:{i:0;s:13:"listitem_open";i:1;a:1:{i:0;i:2;}i:2;i:1844;}i:145;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:1844;}i:146;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:36:" [Markdown Tables](#markdown-tables)";}i:2;i:1850;}i:147;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:1886;}i:148;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:1886;}i:149;a:3:{i:0;s:13:"listitem_open";i:1;a:1:{i:0;i:2;}i:2;i:1886;}i:150;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:1886;}i:151;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:14:" [JSON](#json)";}i:2;i:1892;}i:152;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:1906;}i:153;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:1906;}i:154;a:3:{i:0;s:13:"listitem_open";i:1;a:1:{i:0;i:2;}i:2;i:1906;}i:155;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:1906;}i:156;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:32:" [Freeform Text](#freeform-text)";}i:2;i:1912;}i:157;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:1944;}i:158;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:1944;}i:159;a:3:{i:0;s:13:"listitem_open";i:1;a:1:{i:0;i:2;}i:2;i:1944;}i:160;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:1944;}i:161;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:28:" [Nested Data](#nested-data)";}i:2;i:1950;}i:162;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:1978;}i:163;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:1978;}i:164;a:3:{i:0;s:11:"listo_close";i:1;a:0:{}i:2;i:1978;}i:165;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:1978;}i:166;a:3:{i:0;s:13:"listitem_open";i:1;a:1:{i:0;i:1;}i:2;i:1978;}i:167;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:1978;}i:168;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:24:" [Citations](#citations)";}i:2;i:1982;}i:169;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:2006;}i:170;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:2006;}i:171;a:3:{i:0;s:13:"listitem_open";i:1;a:1:{i:0;i:1;}i:2;i:2006;}i:172;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:2006;}i:173;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:54:" [Programmatic Consumption](#programmatic-consumption)";}i:2;i:2010;}i:174;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:2064;}i:175;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:2064;}i:176;a:3:{i:0;s:13:"listitem_open";i:1;a:2:{i:0;i:1;i:1;i:1;}i:2;i:2064;}i:177;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:2064;}i:178;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:38:" [Chain of Thought](#chain-of-thought)";}i:2;i:2068;}i:179;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:2106;}i:180;a:3:{i:0;s:10:"listo_open";i:1;a:0:{}i:2;i:2106;}i:181;a:3:{i:0;s:13:"listitem_open";i:1;a:1:{i:0;i:2;}i:2;i:2106;}i:182;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:2106;}i:183;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:24:" [Averaging](#averaging)";}i:2;i:2112;}i:184;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:2136;}i:185;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:2136;}i:186;a:3:{i:0;s:13:"listitem_open";i:1;a:1:{i:0;i:2;}i:2;i:2136;}i:187;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:2136;}i:188;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:40:" [Interpreting Code](#interpreting-code)";}i:2;i:2142;}i:189;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:2182;}i:190;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:2182;}i:191;a:3:{i:0;s:13:"listitem_open";i:1;a:1:{i:0;i:2;}i:2;i:2182;}i:192;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:2182;}i:193;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:26:" [Delimiters](#delimiters)";}i:2;i:2188;}i:194;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:2214;}i:195;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:2214;}i:196;a:3:{i:0;s:11:"listo_close";i:1;a:0:{}i:2;i:2214;}i:197;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:2214;}i:198;a:3:{i:0;s:13:"listitem_open";i:1;a:2:{i:0;i:1;i:1;i:1;}i:2;i:2214;}i:199;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:2214;}i:200;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:28:" [Fine Tuning](#fine-tuning)";}i:2;i:2218;}i:201;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:2246;}i:202;a:3:{i:0;s:10:"listo_open";i:1;a:0:{}i:2;i:2246;}i:203;a:3:{i:0;s:13:"listitem_open";i:1;a:1:{i:0;i:2;}i:2;i:2246;}i:204;a:3:{i:0;s:16:"listcontent_open";i:1;a:0:{}i:2;i:2246;}i:205;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:24:" [Downsides](#downsides)";}i:2;i:2252;}i:206;a:3:{i:0;s:17:"listcontent_close";i:1;a:0:{}i:2;i:2276;}i:207;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:2276;}i:208;a:3:{i:0;s:11:"listo_close";i:1;a:0:{}i:2;i:2276;}i:209;a:3:{i:0;s:14:"listitem_close";i:1;a:0:{}i:2;i:2276;}i:210;a:3:{i:0;s:11:"listo_close";i:1;a:0:{}i:2;i:2276;}i:211;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:2276;}i:212;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:47:"- [Additional Resources](#additional-resources)";}i:2;i:2277;}i:213;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:2324;}i:214;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:2324;}i:215;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:40:"## What is a Large Language Model (LLM)?";}i:2;i:2326;}i:216;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:2366;}i:217;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:2366;}i:218;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:324:"A large language model is a prediction engine that takes a sequence of words
and tries to predict the most likely sequence to come after that sequence[^1].
It does this by assigning a probability to likely next sequences and then
samples from those to choose one[^2]. The process repeats until some stopping
criteria is met.";}i:2;i:2368;}i:219;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:2692;}i:220;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:2692;}i:221;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:441:"Large language models learn these probabilities by training on large corpuses
of text. A consequence of this is that the models will cater to some use cases
better than others (e.g. if it’s trained on GitHub data, it’ll understand the
probabilities of sequences in source code really well). Another consequence is
that the model may generate statements that seem plausible, but are actually
just random without being grounded in reality.";}i:2;i:2694;}i:222;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:3135;}i:223;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:3135;}i:224;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:100:"As language models become more accurate at predicting sequences, [many
surprising abilities
emerge](";}i:2;i:3137;}i:225;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:76:"https://www.assemblyai.com/blog/emergent-abilities-of-large-language-models/";i:1;N;}i:2;i:3237;}i:226;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:").";}i:2;i:3313;}i:227;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:3315;}i:228;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:3315;}i:229;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:241:"[^1]: Language models actually use tokens, not words. A token roughly maps to a syllable in a word, or about 4 characters.
[^2]: There are many different pruning and sampling strategies to alter the behavior and performance of the sequences.";}i:2;i:3317;}i:230;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:3558;}i:231;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:3558;}i:232;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:74:"### A Brief, Incomplete, and Somewhat Incorrect History of Language Models";}i:2;i:3560;}i:233;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:3635;}i:234;a:3:{i:0;s:10:"quote_open";i:1;a:0:{}i:2;i:3635;}i:235;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:75:" :pushpin: Skip [to here](#what-is-a-prompt) if you'd like to jump past the";}i:2;i:3637;}i:236;a:3:{i:0;s:9:"linebreak";i:1;a:0:{}i:2;i:3712;}i:237;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:75:" history of language models. This section is for the curious minded, though";}i:2;i:3714;}i:238;a:3:{i:0;s:9:"linebreak";i:1;a:0:{}i:2;i:3789;}i:239;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:75:" may also help you understand the reasoning behind the advice that follows.";}i:2;i:3791;}i:240;a:3:{i:0;s:11:"quote_close";i:1;a:0:{}i:2;i:3866;}i:241;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:3866;}i:242;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:17:"#### Pre-2000’s";}i:2;i:3868;}i:243;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:3885;}i:244;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:3885;}i:245;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:18:"[Language models](";}i:2;i:3887;}i:246;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:56:"https://en.wikipedia.org/wiki/Language_model#Model_types";i:1;N;}i:2;i:3905;}i:247;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:85:")
have existed for decades, though traditional language models (e.g. [n-gram
models](";}i:2;i:3961;}i:248;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:51:"https://en.wikipedia.org/wiki/N-gram_language_model";i:1;N;}i:2;i:4046;}i:249;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:97:")) have many
deficiencies in terms of an explosion of state space ([the curse of
dimensionality](";}i:2;i:4097;}i:250;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:53:"https://en.wikipedia.org/wiki/Curse_of_dimensionality";i:1;N;}i:2;i:4194;}i:251;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:371:")) and
working with novel phrases that they’ve never seen (sparsity). Plainly, older
language models can generate text that vaguely resembles the statistics of
human generated text, but there is no consistency within the output – and a
reader will quickly realize it’s all gibberish. N-gram models also don’t scale
to large values of N, so are inherently limited.";}i:2;i:4247;}i:252;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:4618;}i:253;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:4618;}i:254;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:17:"#### Mid-2000’s";}i:2;i:4620;}i:255;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:4637;}i:256;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:4637;}i:257;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:150:"In 2007, Geoffrey Hinton – famous for popularizing backpropagation in 1980’s –
[published an important advancement in training neural
networks](";}i:2;i:4639;}i:258;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:47:"http://www.cs.toronto.edu/~fritz/absps/tics.pdf";i:1;N;}i:2;i:4789;}i:259;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:513:") that unlocked much
deeper networks. Applying these simple deep neural networks to language
modeling helped alleviate some of problems with language models – they
represented nuanced arbitrary concepts in a finite space and continuous way,
gracefully handling sequences not seen in the training corpus. These simple
neural networks learned the probabilities of their training corpus well, but
the output would statistically match the training data and generally not be
coherent relative to the input sequence. ";}i:2;i:4836;}i:260;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:5349;}i:261;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:5349;}i:262;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:19:"#### Early-2010’s";}i:2;i:5351;}i:263;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:5370;}i:264;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:5370;}i:265;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:86:"Although they were first introduced in 1995, [Long Short-Term Memory (LSTM)
Networks](";}i:2;i:5372;}i:266;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:52:"https://en.wikipedia.org/wiki/Long_short-term_memory";i:1;N;}i:2;i:5458;}i:267;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:356:") found their
time to shine in the 2010’s. LSTMs allowed models to process arbitrary length
sequences and, importantly, alter their internal state dynamically as they
processed the input to remember previous things they saw. This minor tweak led
to remarkable improvements. In 2015, Andrej Karpathy [famously wrote about
creating a character-level
lstm](";}i:2;i:5510;}i:268;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:55:"http://karpathy.github.io/2015/05/21/rnn-effectiveness/";i:1;N;}i:2;i:5866;}i:269;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:53:") that performed
far better than it had any right to.";}i:2;i:5921;}i:270;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:5974;}i:271;a:3:{i:0;s:6:"p_open";i:1;a:0:{}i:2;i:5974;}i:272;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:197:"LSTMs have seemingly magical abilities, but struggle with long term
dependencies. If you asked it to complete the sentence, “In France, we
traveled around, ate many pastries, drank lots of wine, ";}i:2;i:5976;}i:273;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"...";}i:2;i:6173;}i:274;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:16:" lots more text ";}i:2;i:6176;}i:275;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"...";}i:2;i:6192;}i:276;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:34:"
, but never learned how to speak ";}i:2;i:6195;}i:277;a:3:{i:0;s:14:"underline_open";i:1;a:0:{}i:2;i:6229;}i:278;a:3:{i:0;s:15:"underline_close";i:1;a:0:{}i:2;i:6231;}i:279;a:3:{i:0;s:14:"underline_open";i:1;a:0:{}i:2;i:6233;}i:280;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:300:"_”, the model might struggle with
predicting “French”. They also process input one token at a time, so are
inherently sequential, slow to train, and the `Nth` token only knows about the
`N - 1` tokens prior to it.

#### Late-2010’s

In 2017, Google wrote a paper, [Attention Is All You
Need](";}i:2;i:6235;}i:281;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:36:"https://arxiv.org/pdf/1706.03762.pdf";i:1;N;}i:2;i:6535;}i:282;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:42:"), that introduced [Transformer
Networks](";}i:2;i:6571;}i:283;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:42:"https://en.wikipedia.org/wiki/Transformer_";i:1;N;}i:2;i:6613;}i:284;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1136:"(machine_learning_model))
and kicked off a massive revolution in natural language processing. Overnight,
machines could suddenly do tasks like translating between languages nearly as
good as (sometimes better than) humans. Transformers are highly parallelizable
and introduce a mechanism, called “attention”, for the model to efficiently
place emphasis on specific parts of the input. Transformers analyze the entire
input all at once, in parallel, choosing which parts are most important and
influential. Every output token is influenced by every input token.

Transformers are highly parallelizable, efficient to train, and produce
astounding results. A downside to transformers is that they have a fixed input
and output size – the context window – and computation increases
quadratically with the size of this window (in some cases, memory does as
well!) [^3].

Transformers are not the end of the road, but the vast majority of recent
improvements in natural language processing have involved them. There is still
abundant active research on various ways of implementing and applying them,
such as [Amazon’s AlexaTM
20B](";}i:2;i:6655;}i:285;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:93:"https://www.amazon.science/blog/20b-parameter-alexa-model-sets-new-marks-in-few-shot-learning";i:1;N;}i:2;i:7791;}i:286;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:527:")
which outperforms GPT-3 in a number of tasks and is an order of magnitude
smaller in its number of parameters.

[^3]: There are more recent variations to make these more compute and memory efficient, but remains an active area of research.

#### 2020’s

While technically starting in 2018, the theme of the 2020’s has been
Generative Pre-Trained models – more famously known as GPT. One
year after the “Attention Is All You Need” paper, OpenAI released [Improving
Language Understanding by Generative
Pre-Training](";}i:2;i:7884;}i:287;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:119:"https://s3-us-west-2.amazonaws.com/openai-assets/research-covers/language-unsupervised/language_understanding_paper.pdf";i:1;N;}i:2;i:8411;}i:288;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:365:").
This paper established that you can train a large language model on a massive
set of data without any specific agenda, and then once the model has learned
the general aspects of language, you can fine-tune it for specific tasks and
quickly get state-of-the-art results.

In 2020, OpenAI followed up with their GPT-3 paper [Language Models are
Few-Shot
Learners](";}i:2;i:8530;}i:289;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:89:"https://proceedings.neurips.cc/paper/2020/file/1457c0d6bfcb4967418bfb8ac142f64a-Paper.pdf";i:1;N;}i:2;i:8895;}i:290;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:385:"),
showing that if you scale up GPT-like models by another factor of ~10x, in
terms of number of parameters and quantity of training data, you no
longer have to fine-tune it for many tasks. The capabilities emerge naturally
and you get state-of-the-art results via text interaction with the model.

In 2022, OpenAI followed-up on their GPT-3 accomplishments by releasing
[InstructGPT](";}i:2;i:8984;}i:291;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:49:"https://openai.com/research/instruction-following";i:1;N;}i:2;i:9369;}i:292;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:201:"). The intent
here was to tweak the model to follow instructions, while also being less
toxic and biased in its outputs. The key ingredient here was [Reinforcement
Learning from Human Feedback (RLHF)](";}i:2;i:9418;}i:293;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:36:"https://arxiv.org/pdf/1706.03741.pdf";i:1;N;}i:2;i:9619;}i:294;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:238:"), a
concept co-authored by Google and OpenAI in 2017[^4], which allows humans to
be in the training loop to fine-tune the model output to be more in line with
human preferences. InstructGPT is the predecessor to the now famous
[ChatGPT](";}i:2;i:9655;}i:295;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:37:"https://en.wikipedia.org/wiki/ChatGPT";i:1;N;}i:2;i:9893;}i:296;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:140:").

OpenAI has been a major contributor to large language models over the last few
years, including the most recent introduction of
[GPT-4](";}i:2;i:9930;}i:297;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:39:"https://cdn.openai.com/papers/gpt-4.pdf";i:1;N;}i:2;i:10070;}i:298;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:97:"), but they are not alone. Meta
has introduced many open source large language models like
[OPT](";}i:2;i:10109;}i:299;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:39:"https://huggingface.co/facebook/opt-66b";i:1;N;}i:2;i:10206;}i:300;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:13:"),
[OPT-IML](";}i:2;i:10245;}i:301;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:43:"https://huggingface.co/facebook/opt-iml-30b";i:1;N;}i:2;i:10258;}i:302;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:35:") (instruction tuned),
and [LLaMa](";}i:2;i:10301;}i:303;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:64:"https://ai.facebook.com/blog/large-language-model-llama-meta-ai/";i:1;N;}i:2;i:10336;}i:304;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:41:").
Google released models like
[FLAN-T5](";}i:2;i:10400;}i:305;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:41:"https://huggingface.co/google/flan-t5-xxl";i:1;N;}i:2;i:10441;}i:306;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:13:") and
[BERT](";}i:2;i:10482;}i:307;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:40:"https://huggingface.co/bert-base-uncased";i:1;N;}i:2;i:10495;}i:308;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:84:"). And there is a huge open
source research community releasing models like
[BLOOM](";}i:2;i:10535;}i:309;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:39:"https://huggingface.co/bigscience/bloom";i:1;N;}i:2;i:10619;}i:310;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:17:") and
[StableLM](";}i:2;i:10658;}i:311;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:41:"https://github.com/stability-AI/stableLM/";i:1;N;}i:2;i:10675;}i:312;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:717:").

Progress is now moving so swiftly that every few weeks the state-of-the-art is
changing or models that previously required clusters to run now run on
Raspberry PIs.

[^4]: 2017 was a big year for natural language processing.

## What is a prompt?

A prompt, sometimes referred to as context, is the text provided to a
model before it begins generating output. It guides the model to explore a
particular area of what it has learned so that the output is relevant to your
goals. As an analogy, if you think of the language model as a source code
interpreter, then a prompt is the source code to be interpreted. Somewhat
amusingly, a language model will happily attempt to guess what source code
will do:

<p align=";}i:2;i:10716;}i:313;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:11433;}i:314;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"center";}i:2;i:11434;}i:315;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:11440;}i:316;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:">
  <img width=";}i:2;i:11441;}i:317;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:11456;}i:318;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"450";}i:2;i:11457;}i:319;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:11460;}i:320;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:" src=";}i:2;i:11461;}i:321;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:11466;}i:322;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/231946874-be91d3de-d773-4a6c-a4ea-21043bd5fc13.png";i:1;N;}i:2;i:11467;}i:323;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:11565;}i:324;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:" title=";}i:2;i:11566;}i:325;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:11573;}i:326;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:41:"The GPT-4 model interpreting Python code.";}i:2;i:11574;}i:327;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:11615;}i:328;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:133:">
</p>

And it *almost* interprets the Python perfectly!

Frequently, prompts will be an instruction or a question, like:

 <p align=";}i:2;i:11616;}i:329;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:11749;}i:330;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"center";}i:2;i:11750;}i:331;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:11756;}i:332;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:">
  <img width=";}i:2;i:11757;}i:333;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:11772;}i:334;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"500";}i:2;i:11773;}i:335;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:11776;}i:336;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:" src=";}i:2;i:11777;}i:337;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:11782;}i:338;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/232413246-81db18dc-ef5b-4073-9827-77bd0317d031.png";i:1;N;}i:2;i:11783;}i:339;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:11881;}i:340;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:127:">
</p>

On the other hand, if you don’t specify a prompt, the model has no anchor to
work from and you’ll see that it just ";}i:2;i:11882;}i:341;a:3:{i:0;s:11:"strong_open";i:1;a:0:{}i:2;i:12009;}i:342;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:51:"randomly samples from anything it has
ever consumed";}i:2;i:12011;}i:343;a:3:{i:0;s:12:"strong_close";i:1;a:0:{}i:2;i:12062;}i:344;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:":

";}i:2;i:12064;}i:345;a:3:{i:0;s:11:"strong_open";i:1;a:0:{}i:2;i:12067;}i:346;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:19:"From GPT-3-Davinci:";}i:2;i:12069;}i:347;a:3:{i:0;s:12:"strong_close";i:1;a:0:{}i:2;i:12088;}i:348;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:13:"

| ![image](";}i:2;i:12090;}i:349;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/232413846-70b05cd1-31b6-4977-93f0-20bf29af7132.png";i:1;N;}i:2;i:12103;}i:350;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:13:") | ![image](";}i:2;i:12201;}i:351;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/232413930-7d414dcd-87e5-431a-91c8-bb6e0ef54f42.png";i:1;N;}i:2;i:12214;}i:352;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:13:") | ![image](";}i:2;i:12312;}i:353;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/232413978-59c7f47d-ec20-4673-9458-85471a41fee0.png";i:1;N;}i:2;i:12325;}i:354;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:") |
| ";}i:2;i:12423;}i:355;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:12429;}i:356;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:" | ";}i:2;i:12432;}i:357;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:12435;}i:358;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:" | ";}i:2;i:12438;}i:359;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:12441;}i:360;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:4:" |

";}i:2;i:12444;}i:361;a:3:{i:0;s:11:"strong_open";i:1;a:0:{}i:2;i:12448;}i:362;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:11:"From GPT-4:";}i:2;i:12450;}i:363;a:3:{i:0;s:12:"strong_close";i:1;a:0:{}i:2;i:12461;}i:364;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:"
| ![image](";}i:2;i:12463;}i:365;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/232414631-928955e5-3bab-4d57-b1d6-5e56f00ffda1.png";i:1;N;}i:2;i:12475;}i:366;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:13:") | ![image](";}i:2;i:12573;}i:367;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/232414678-e5b6d3f4-36c6-420f-b38f-2f9c8df391fb.png";i:1;N;}i:2;i:12586;}i:368;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:13:") | ![image](";}i:2;i:12684;}i:369;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/232414734-c8f09cad-aceb-4149-a28a-33675cde8011.png";i:1;N;}i:2;i:12697;}i:370;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:") |
| ";}i:2;i:12795;}i:371;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:12801;}i:372;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:" | ";}i:2;i:12804;}i:373;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:12807;}i:374;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:" | ";}i:2;i:12810;}i:375;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:12813;}i:376;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:652:" |

### Hidden Prompts

> :warning: Always assume that any content in a hidden prompt can be seen by the user.

In applications where a user is interacting with a model dynamically, such as
chatting with the model, there will typically be portions of the prompt that
are never intended to be seen by the user. These hidden portions may occur
anywhere, though there is almost always a hidden prompt at the start of a
conversation.

Typically, this includes an initial chunk of text that sets the tone, model
constraints, and goals, along with other dynamic information that is specific
to the particular session – user name, location, time of day, etc";}i:2;i:12816;}i:377;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"...";}i:2;i:13468;}i:378;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:187:"

The model is static and frozen at a point in time, so if you want it to know
current information, like the time or the weather, you must provide it.

If you’re using [the OpenAI Chat
";}i:2;i:13471;}i:379;a:3:{i:0;s:7:"acronym";i:1;a:1:{i:0;s:3:"API";}i:2;i:13658;}i:380;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:"](";}i:2;i:13661;}i:381;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:57:"https://platform.openai.com/docs/guides/chat/introduction";i:1;N;}i:2;i:13663;}i:382;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:184:"), they
delineate hidden prompt content by placing it in the `system` role.

Here’s an example of a hidden prompt followed by interactions with the content
in that prompt:

<p align=";}i:2;i:13720;}i:383;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:13904;}i:384;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"center";}i:2;i:13905;}i:385;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:13911;}i:386;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:">
  <img width=";}i:2;i:13912;}i:387;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:13927;}i:388;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"550";}i:2;i:13928;}i:389;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:13931;}i:390;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:" src=";}i:2;i:13932;}i:391;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:13937;}i:392;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/232416074-84ebcc10-2dfc-49e1-9f48-a240102877ee.png";i:1;N;}i:2;i:13938;}i:393;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:14036;}i:394;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:" title=";}i:2;i:14037;}i:395;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:14044;}i:396;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:29:" A very simple hidden prompt.";}i:2;i:14045;}i:397;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:14074;}i:398;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:326:">
</p>

In this example, you can see we explain to the bot the various roles, some
context on the user, some dynamic data we want the bot to have access to, and
then guidance on how the bot should respond.

In practice, hidden prompts may be quite large. Here’s a larger prompt taken
from a [ChatGPT command-line
assistant](";}i:2;i:14075;}i:399;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:76:"https://github.com/manno/chatgpt-linux-assistant/blob/main/system_prompt.txt";i:1;N;}i:2;i:14401;}i:400;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:31:"):

<details>
  <summary>From: ";}i:2;i:14477;}i:401;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:48:"https://github.com/manno/chatgpt-linux-assistant";i:1;N;}i:2;i:14508;}i:402;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:71:" </summary>

```
We are a in a chatroom with 3 users. 1 user is called ";}i:2;i:14556;}i:403;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:14627;}i:404;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"Human";}i:2;i:14628;}i:405;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:14633;}i:406;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:22:", the other is called ";}i:2;i:14634;}i:407;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:14656;}i:408;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"Backend";}i:2;i:14657;}i:409;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:14664;}i:410;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:25:" and the other is called ";}i:2;i:14665;}i:411;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:14690;}i:412;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:32:"Proxy Natural Language Processor";}i:2;i:14691;}i:413;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:14723;}i:414;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:19:". I will type what ";}i:2;i:14724;}i:415;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:14743;}i:416;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"Human";}i:2;i:14744;}i:417;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:14749;}i:418;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:" says and what ";}i:2;i:14750;}i:419;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:14765;}i:420;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"Backend";}i:2;i:14766;}i:421;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:14773;}i:422;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:28:" replies. You will act as a ";}i:2;i:14774;}i:423;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:14802;}i:424;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:32:"Proxy Natural Language Processor";}i:2;i:14803;}i:425;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:14835;}i:426;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:30:" to forward the requests that ";}i:2;i:14836;}i:427;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:14866;}i:428;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"Human";}i:2;i:14867;}i:429;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:14872;}i:430;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:39:" asks for in a JSON format to the user ";}i:2;i:14873;}i:431;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:14912;}i:432;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"Backend";}i:2;i:14913;}i:433;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:14920;}i:434;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:". User ";}i:2;i:14921;}i:435;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:14928;}i:436;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"Backend";}i:2;i:14929;}i:437;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:14936;}i:438;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:171:" is an Ubuntu server and the strings that are sent to it are ran in a shell and then it replies with the command STDOUT and the exit code. The Ubuntu server is mine. When ";}i:2;i:14937;}i:439;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:15108;}i:440;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"Backend";}i:2;i:15109;}i:441;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:15116;}i:442;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:44:" replies with the STDOUT and exit code, you ";}i:2;i:15117;}i:443;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:15161;}i:444;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:32:"Proxy Natural Language Processor";}i:2;i:15162;}i:445;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:15194;}i:446;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:83:" will parse and format that data into a simple English friendly way and send it to ";}i:2;i:15195;}i:447;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:15278;}i:448;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"Human";}i:2;i:15279;}i:449;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:15284;}i:450;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:165:". Here is an example:

I ask as human:
Human: How many unedited videos are left?
Then you send a command to the Backend:
Proxy Natural Language Processor: @Backend {";}i:2;i:15285;}i:451;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:15450;}i:452;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"command";}i:2;i:15451;}i:453;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:15458;}i:454;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1:":";}i:2;i:15459;}i:455;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:15460;}i:456;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:46:"find ./Videos/Unedited/ -iname '*.mp4' | wc -l";}i:2;i:15461;}i:457;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:15507;}i:458;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:77:"}
Then the backend responds with the command STDOUT and exit code:
Backend: {";}i:2;i:15508;}i:459;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:15585;}i:460;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"STDOUT";}i:2;i:15586;}i:461;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:15592;}i:462;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1:":";}i:2;i:15593;}i:463;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:15594;}i:464;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1:"5";}i:2;i:15595;}i:465;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:15596;}i:466;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:", ";}i:2;i:15597;}i:467;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:15599;}i:468;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:8:"EXITCODE";}i:2;i:15600;}i:469;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:15608;}i:470;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1:":";}i:2;i:15609;}i:471;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:15610;}i:472;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1:"0";}i:2;i:15611;}i:473;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:15612;}i:474;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:122:"}
Then you reply to the user:
Proxy Natural Language Processor: @Human There are 5 unedited videos left.

Only reply what ";}i:2;i:15613;}i:475;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:15735;}i:476;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:32:"Proxy Natural Language Processor";}i:2;i:15736;}i:477;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:15768;}i:478;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:233:" is supposed to say and nothing else. Not now nor in the future for any reason.

Another example:

I ask as human:
Human: What is a PEM certificate?
Then you send a command to the Backend:
Proxy Natural Language Processor: @Backend {";}i:2;i:15769;}i:479;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:16002;}i:480;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"command";}i:2;i:16003;}i:481;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:16010;}i:482;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1:":";}i:2;i:16011;}i:483;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:16012;}i:484;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:10:"xdg-open '";}i:2;i:16013;}i:485;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:51:"https://en.wikipedia.org/wiki/Privacy-Enhanced_Mail";i:1;N;}i:2;i:16023;}i:486;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1:"'";}i:2;i:16074;}i:487;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:16075;}i:488;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:77:"}
Then the backend responds with the command STDOUT and exit code:
Backend: {";}i:2;i:16076;}i:489;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:16153;}i:490;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"STDOUT";}i:2;i:16154;}i:491;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:16160;}i:492;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1:":";}i:2;i:16161;}i:493;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:16162;}i:494;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:16163;}i:495;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:", ";}i:2;i:16164;}i:496;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:16166;}i:497;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:8:"EXITCODE";}i:2;i:16167;}i:498;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:16175;}i:499;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1:":";}i:2;i:16176;}i:500;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:16177;}i:501;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1:"0";}i:2;i:16178;}i:502;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:16179;}i:503;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:153:"}
Then you reply to the user:
Proxy Natural Language Processor: @Human I have opened a link which describes what a PEM certificate is.


Only reply what ";}i:2;i:16180;}i:504;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:16333;}i:505;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:32:"Proxy Natural Language Processor";}i:2;i:16334;}i:506;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:16366;}i:507;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:372:" is supposed to say and nothing else. Not now nor in the future for any reason.

Do NOT REPLY as Backend. DO NOT complete what Backend is supposed to reply. YOU ARE NOT TO COMPLETE what Backend is supposed to reply.
Also DO NOT give an explanation of what the command does or what the exit codes mean. DO NOT EVER, NOW OR IN THE FUTURE, REPLY AS BACKEND.

Only reply what ";}i:2;i:16367;}i:508;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:16739;}i:509;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:32:"Proxy Natural Language Processor";}i:2;i:16740;}i:510;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:16772;}i:511;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:870:" is supposed to say and nothing else. Not now nor in the future for any reason.
```
</details>

You’ll see some good practices there, such as including lots of examples,
repetition for important behavioral aspects, constraining the replies, etc…

> :warning: Always assume that any content in a hidden prompt can be seen by the user.

### Tokens

If you thought tokens were :fire: in 2022, tokens in 2023 are on a whole
different plane of existence. The atomic unit of consumption for a language
model is not a “word”, but rather a “token”. You can kind of think of tokens
as syllables, and on average they work out to about 750 words per 1,000
tokens. They represent many concepts beyond just alphabetical characters –
such as punctuation, sentence boundaries, and the end of a document.

Here’s an example of how GPT may tokenize a sequence:

<p align=";}i:2;i:16773;}i:512;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:17643;}i:513;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"center";}i:2;i:17644;}i:514;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:17650;}i:515;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:">
  <img width=";}i:2;i:17651;}i:516;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:17666;}i:517;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"550";}i:2;i:17667;}i:518;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:17670;}i:519;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:" src=";}i:2;i:17671;}i:520;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:17676;}i:521;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/232417569-8d562792-64b5-423d-a7a2-db7513dd4d61.png";i:1;N;}i:2;i:17677;}i:522;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:17775;}i:523;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:" title=";}i:2;i:17776;}i:524;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:17783;}i:525;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:50:"An example tokenization. You can experiment here: ";}i:2;i:17784;}i:526;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:37:"https://platform.openai.com/tokenizer";i:1;N;}i:2;i:17834;}i:527;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1:" ";}i:2;i:17871;}i:528;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:17872;}i:529;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:51:">
</p>

You can experiment with a tokenizer here: [";}i:2;i:17873;}i:530;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:38:"https://platform.openai.com/tokenizer]";i:1;N;}i:2;i:17924;}i:531;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1:"(";}i:2;i:17962;}i:532;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:37:"https://platform.openai.com/tokenizer";i:1;N;}i:2;i:17963;}i:533;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:546:")

Different models will use different tokenizers with different levels of granularity. You could, in theory, just feed a model 0’s and 1’s – but then the model needs to learn the concept of characters from bits, and then the concept of words from characters, and so forth. Similarly, you could feed the model a stream of raw characters, but then the model needs to learn the concept of words, and punctuation, etc… and, in general, the models will perform worse.

To learn more, [Hugging Face has a wonderful introduction to tokenizers](";}i:2;i:18000;}i:534;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:58:"https://huggingface.co/docs/transformers/tokenizer_summary";i:1;N;}i:2;i:18546;}i:535;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:378:") and why they need to exist.

There’s a lot of nuance around tokenization, such as vocabulary size or different languages treating sentence structure meaningfully different (e.g. words not being separated by spaces). Fortunately, language model APIs will almost always take raw text as input and tokenize it behind the scenes – *so you rarely need to think about tokens*.

";}i:2;i:18604;}i:536;a:3:{i:0;s:11:"strong_open";i:1;a:0:{}i:2;i:18982;}i:537;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:71:"Except for one important scenario, which we discuss next: token limits.";}i:2;i:18984;}i:538;a:3:{i:0;s:12:"strong_close";i:1;a:0:{}i:2;i:19055;}i:539;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1561:"

### Token Limits

Prompts tend to be append-only, because you want the bot to have the entire context of previous messages in the conversation. Language models, in general, are stateless and won’t remember anything about previous requests to them, so you always need to include everything that it might need to know that is specific to the current session.

A major downside of this is that the leading language model architecture, the Transformer, has a fixed input and output size – at a certain point the prompt can’t grow any larger. The total size of the prompt, sometimes referred to as the “context window”, is model dependent. For GPT-3, it is 4,096 tokens. For GPT-4, it is 8,192 tokens or 32,768 tokens depending on which variant you use.

If your context grows too large for the model, the most common tactic is the truncate the context in a sliding window fashion. If you think of a prompt as `hidden initialization prompt + messages[]`, usually the hidden prompt will remain unaltered, and the `messages[]` array will take the last N messages.

You may also see more clever tactics for prompt truncation – such as
discarding only the user messages first, so that the bot's previous answers
stay in the context for as long as possible, or asking an LLM to summarize the
conversation and then replacing all of the messages with a single message
containing that summary. There is no correct answer here and the solution will
depend on your application.

Importantly, when truncating the context, you must truncate aggressively enough to ";}i:2;i:19057;}i:540;a:3:{i:0;s:11:"strong_open";i:1;a:0:{}i:2;i:20618;}i:541;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:35:"allow room for the response as well";}i:2;i:20620;}i:542;a:3:{i:0;s:12:"strong_close";i:1;a:0:{}i:2;i:20655;}i:543;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:392:". OpenAI’s token limits include both the length of the input and the length of the output. If your input to GPT-3 is 4,090 tokens, it can only generate 6 tokens in response.

> 🧙‍♂️ If you’d like to count the number of tokens before sending the raw text to the model, the specific tokenizer to use will depend on which model you are using. OpenAI has a library called [tiktoken](";}i:2;i:20657;}i:544;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:54:"https://github.com/openai/tiktoken/blob/main/README.md";i:1;N;}i:2;i:21049;}i:545;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1023:") that you can use with their models – though there is an important caveat that their internal tokenizer may vary slightly in count, and they may append other metadata, so consider this an approximation.
> 
> If you’d like an approximation without having access to a tokenizer, `input.length / 4` will give a rough, but better than you’d expect, approximation for English inputs.

### Prompt Hacking

Prompt engineering and large language models are a fairly nascent field, so new ways to hack around them are being discovered every day. The two large classes of attacks are:

1. Make the bot bypass any guidelines you have given it.
2. Make the bot output hidden context that you didn’t intend for the user to see.

There are no known mechanisms to comprehensively stop these, so it is important that you assume the bot may do or say anything when interacting with an adversarial user. Fortunately, in practice, these are mostly cosmetic concerns.

Think of prompts as a way to improve the normal user experience. ";}i:2;i:21103;}i:546;a:3:{i:0;s:11:"strong_open";i:1;a:0:{}i:2;i:22126;}i:547;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:183:"We design prompts so that normal users don’t stumble outside of our intended interactions – but always assume that a determined user will be able to bypass our prompt constraints.";}i:2;i:22128;}i:548;a:3:{i:0;s:12:"strong_close";i:1;a:0:{}i:2;i:22311;}i:549;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:429:"

#### Jailbreaks

Typically hidden prompts will tell the bot to behave with a certain persona and focus on specific tasks or avoid certain words. It is generally safe to assume the bot will follow these guidelines for non-adversarial users, although non-adversarial users may accidentally bypass the guidelines too.

For  example, we can tell the bot:

```
You are a helpful assistant, but you are never allowed to use the word ";}i:2;i:22313;}i:550;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:22742;}i:551;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:8:"computer";}i:2;i:22743;}i:552;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:22751;}i:553;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:181:".
```

If we then ask it a question about computers, it will refer to them as a “device used for computing” because it isn’t allowed to use the word “computer”.

<p align=";}i:2;i:22752;}i:554;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:22933;}i:555;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"center";}i:2;i:22934;}i:556;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:22940;}i:557;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:">
  <img width=";}i:2;i:22941;}i:558;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:22956;}i:559;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"550";}i:2;i:22957;}i:560;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:22960;}i:561;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:" src=";}i:2;i:22961;}i:562;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:22966;}i:563;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/232420043-ebe5bcf1-25d9-4a31-ba84-13e9e1f62de2.png";i:1;N;}i:2;i:22967;}i:564;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:23065;}i:565;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:" title=";}i:2;i:23066;}i:566;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:23073;}i:567;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:49:"GPT-4 trying hard to not say the word 'computer'.";}i:2;i:23074;}i:568;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:23123;}i:569;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:61:">
</p>

It will absolutely refuse to say the word:

<p align=";}i:2;i:23124;}i:570;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:23185;}i:571;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"center";}i:2;i:23186;}i:572;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:23192;}i:573;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:">
  <img width=";}i:2;i:23193;}i:574;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:23208;}i:575;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"550";}i:2;i:23209;}i:576;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:23212;}i:577;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:" src=";}i:2;i:23213;}i:578;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:23218;}i:579;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/232420306-6fcdd6e2-b107-45d5-a1ee-4132fbb5853e.png";i:1;N;}i:2;i:23219;}i:580;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:23317;}i:581;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:179:">
</p>

But we can bypass these instructions and get the model to happily use the word if we trick it by asking it to translate the pig latin version of “computer”.

<p align=";}i:2;i:23318;}i:582;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:23497;}i:583;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"center";}i:2;i:23498;}i:584;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:23504;}i:585;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:">
  <img width=";}i:2;i:23505;}i:586;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:23520;}i:587;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"550";}i:2;i:23521;}i:588;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:23524;}i:589;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:" src=";}i:2;i:23525;}i:590;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:23530;}i:591;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/232420600-56083a10-b382-46a7-be18-eb9c005b8371.png";i:1;N;}i:2;i:23531;}i:592;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:23629;}i:593;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:51:">
</p>

There are [a number of defensive measures](";}i:2;i:23630;}i:594;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:74:"https://learnprompting.org/docs/prompt_hacking/defensive_measures/overview";i:1;N;}i:2;i:23681;}i:595;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:149:") you can take here, but typically the best bet is to reiterate your most important constraints as close to the end as possible. For the OpenAI chat ";}i:2;i:23755;}i:596;a:3:{i:0;s:7:"acronym";i:1;a:1:{i:0;s:3:"API";}i:2;i:23904;}i:597;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:117:", this might mean including it as a `system` message after the last `user` message. Here’s an example:

| ![image](";}i:2;i:23907;}i:598;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/232421097-adcaace3-0b21-4c1e-a5c8-46bb25faa2f7.png";i:1;N;}i:2;i:24024;}i:599;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:13:") | ![image](";}i:2;i:24122;}i:600;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/232421142-a47e75b4-5ff6-429d-9abd-a78dbc72466e.png";i:1;N;}i:2;i:24135;}i:601;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:") |
| ";}i:2;i:24233;}i:602;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:24239;}i:603;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:" | ";}i:2;i:24242;}i:604;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:24245;}i:605;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:89:" |

Despite OpenAI investing a lot into jailbreaks, there are [very clever work arounds](";}i:2;i:24248;}i:606;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:59:"https://twitter.com/alexalbert__/status/1636488551817965568";i:1;N;}i:2;i:24337;}i:607;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:27:") being [shared every day](";}i:2;i:24396;}i:608;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:55:"https://twitter.com/zswitten/status/1598088267789787136";i:1;N;}i:2;i:24423;}i:609;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:65:").

#### Leaks

If you missed the previous warnings in this doc, ";}i:2;i:24478;}i:610;a:3:{i:0;s:11:"strong_open";i:1;a:0:{}i:2;i:24543;}i:611;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:104:"you should always assume that any data exposed to the language model will eventually be seen by the user";}i:2;i:24545;}i:612;a:3:{i:0;s:12:"strong_close";i:1;a:0:{}i:2;i:24649;}i:613;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:116:".

As part of constructing prompts, you will often embed a bunch of data in hidden prompts (a.k.a. system prompts). ";}i:2;i:24651;}i:614;a:3:{i:0;s:11:"strong_open";i:1;a:0:{}i:2;i:24767;}i:615;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:55:"The bot will happily relay this information to the user";}i:2;i:24769;}i:616;a:3:{i:0;s:12:"strong_close";i:1;a:0:{}i:2;i:24824;}i:617;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:":

<p align=";}i:2;i:24826;}i:618;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:24838;}i:619;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"center";}i:2;i:24839;}i:620;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:24845;}i:621;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:">
  <img width=";}i:2;i:24846;}i:622;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:24861;}i:623;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"550";}i:2;i:24862;}i:624;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:24865;}i:625;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:" src=";}i:2;i:24866;}i:626;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:24871;}i:627;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/232422860-731c1de2-9e77-4957-b257-b0bbda48558c.png";i:1;N;}i:2;i:24872;}i:628;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:24970;}i:629;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:" title=";}i:2;i:24971;}i:630;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:24978;}i:631;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:70:"The bot happily regurgitating the information it knows about the user.";}i:2;i:24979;}i:632;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:25049;}i:633;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:307:">
</p>

Even if you instruct it not to reveal the information, and it obeys those instructions, there are millions of ways to leak data in the hidden prompt.

Here we have an example where the bot should never mention my city, but a simple reframing of the question get’s it to spill the beans.

<p align=";}i:2;i:25050;}i:634;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:25357;}i:635;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"center";}i:2;i:25358;}i:636;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:25364;}i:637;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:">
  <img width=";}i:2;i:25365;}i:638;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:25380;}i:639;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"550";}i:2;i:25381;}i:640;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:25384;}i:641;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:" src=";}i:2;i:25385;}i:642;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:25390;}i:643;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/232423121-76568893-fa42-4ad8-b2bc-e1001327fa1e.png";i:1;N;}i:2;i:25391;}i:644;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:25489;}i:645;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:" title=";}i:2;i:25490;}i:646;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:25497;}i:647;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:108:"The bot refuses to reveal personal information, but we convince it to tell me what city I’m in regardless.";}i:2;i:25498;}i:648;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:25606;}i:649;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:130:">
</p>

Similarly, we get the bot to tell us what word it isn’t allowed to say without ever actually saying the word:

<p align=";}i:2;i:25607;}i:650;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:25737;}i:651;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"center";}i:2;i:25738;}i:652;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:25744;}i:653;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:">
  <img width=";}i:2;i:25745;}i:654;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:25760;}i:655;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"550";}i:2;i:25761;}i:656;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:25764;}i:657;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:" src=";}i:2;i:25765;}i:658;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:25770;}i:659;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/232423283-1718f822-59d0-4d18-9a4d-22dd3a2672c0.png";i:1;N;}i:2;i:25771;}i:660;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:25869;}i:661;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:" title=";}i:2;i:25870;}i:662;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:25877;}i:663;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:123:"Technically, the bot never said 'computer', but I was still able to get it to tell me everything I needed to know about it.";}i:2;i:25878;}i:664;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:26001;}i:665;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:142:">
</p>

You should think of a hidden prompt as a means to make the user experience better or more inline with the persona you’re targeting. ";}i:2;i:26002;}i:666;a:3:{i:0;s:11:"strong_open";i:1;a:0:{}i:2;i:26144;}i:667;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:105:"Never place any information in a prompt that you wouldn’t visually render for someone to read on screen";}i:2;i:26146;}i:668;a:3:{i:0;s:12:"strong_close";i:1;a:0:{}i:2;i:26251;}i:669;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:147:".

## Why do we need prompt engineering?

Up above, we used an analogy of prompts as the “source code” that a language model “interprets”. ";}i:2;i:26253;}i:670;a:3:{i:0;s:11:"strong_open";i:1;a:0:{}i:2;i:26400;}i:671;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:102:"Prompt engineering is the art of writing prompts to get the language model to do what we want it to do";}i:2;i:26402;}i:672;a:3:{i:0;s:12:"strong_close";i:1;a:0:{}i:2;i:26504;}i:673;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:634:" – just like software engineering is the art of writing source code to get computers to do what we want them to do.

When writing good prompts, you have to account for the idiosyncrasies of the model(s) you’re working with. The strategies will vary with the complexity of the tasks. You’ll have to come up with mechanisms to constrain the model to achieve reliable results, incorporate dynamic data that the model can’t be trained on, account for limitations in the model’s training data, design around context limits, and many other dimensions.

There’s an old adage that computers will only do what you tell them to do. ";}i:2;i:26506;}i:674;a:3:{i:0;s:11:"strong_open";i:1;a:0:{}i:2;i:27140;}i:675;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:32:"Throw that advice out the window";}i:2;i:27142;}i:676;a:3:{i:0;s:12:"strong_close";i:1;a:0:{}i:2;i:27174;}i:677;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:955:". Prompt engineering inverts this wisdom. It’s like programming in natural language against a non-deterministic computer that will do anything that you haven’t guided it away from doing. 

There are two broad buckets that prompt engineering approaches fall into.

### Give a Bot a Fish

The “give a bot a fish” bucket is for scenarios when you can explicitly give the bot, in the hidden context, all of the information it needs to do whatever task is requested of it.

For example, if a user loaded up their dashboard and we wanted to show them a quick little friendly message about what task items they have outstanding, we could get the bot to summarize it as

> You have 4 receipts/memos to upload. The most recent is from Target on March 5th, and the oldest is from Blink Fitness on January 17th. Thanks for staying on top of your expenses!

by providing a list of the entire inbox and any other user context we’d like it to have.

<p align=";}i:2;i:27176;}i:678;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:28131;}i:679;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"center";}i:2;i:28132;}i:680;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:28138;}i:681;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:">
  <img width=";}i:2;i:28139;}i:682;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:28154;}i:683;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"550";}i:2;i:28155;}i:684;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:28158;}i:685;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:" src=";}i:2;i:28159;}i:686;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:28164;}i:687;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/233465165-e0c6b266-b347-4128-8eaa-73974e852e45.png";i:1;N;}i:2;i:28165;}i:688;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:28263;}i:689;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:" title=";}i:2;i:28264;}i:690;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:28271;}i:691;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:31:"GPT-3 summarizing a task inbox.";}i:2;i:28272;}i:692;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:28303;}i:693;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:523:">
</p>

Similarly, if you were helping a user book a trip, you could:

- Ask the user their dates and destination.
- Behind the scenes, search for flights and hotels.
- Embed the flight and hotel search results in the hidden context.
- Also embed the company’s travel policy in the hidden context.

And then the bot will have real-time travel information + constraints that it
can use to answer questions for the user. Here’s an example of the bot
recommending options, and the user asking it to refine them:

<p align=";}i:2;i:28304;}i:694;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:28827;}i:695;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"center";}i:2;i:28828;}i:696;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:28834;}i:697;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:">
  <img width=";}i:2;i:28835;}i:698;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:28850;}i:699;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"550";}i:2;i:28851;}i:700;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:28854;}i:701;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:" src=";}i:2;i:28855;}i:702;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:28860;}i:703;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/233465425-9e06320c-b6d9-40ef-b5a4-c556861c1328.png";i:1;N;}i:2;i:28861;}i:704;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:28959;}i:705;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:" title=";}i:2;i:28960;}i:706;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:28967;}i:707;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:33:"GPT-4 helping a user book a trip.";}i:2;i:28968;}i:708;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:29001;}i:709;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:691:">
</p>
<details>

  <summary>(Full prompt)</summary>

```
Brex is a platform for managing business expenses. 

The following is a travel expense policy on Brex:

- Airline highest fare class for flights under 6 hours is economy.
- Airline highest fare class for flights over 6 hours is premium economy.
- Car rentals must have an average daily rate of $75 or under.
- Lodging must have an average nightly rate of $400 or under.
- Lodging must be rated 4 stars or higher.
- Meals from restaurants, food delivery, grocery, bars & nightlife must be under $75
- All other expenses must be under $5,000.
- Reimbursements require review.

The hotel options are:
| Hotel Name | Price | Reviews |
| ";}i:2;i:29002;}i:710;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:29693;}i:711;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:" | ";}i:2;i:29696;}i:712;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:29699;}i:713;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:" | ";}i:2;i:29702;}i:714;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:29705;}i:715;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:299:" |
| Hilton Financial District | $109/night | 3.9 stars |
| Hotel VIA | $131/night | 4.4 stars |
| Hyatt Place San Francisco | $186/night | 4.2 stars |
| Hotel Zephyr | $119/night | 4.1 stars review |

The flight options are:
| Airline | Flight Time | Duration | Number of Stops | Class | Price |
| ";}i:2;i:29708;}i:716;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:30007;}i:717;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:" | ";}i:2;i:30010;}i:718;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:30013;}i:719;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:" | ";}i:2;i:30016;}i:720;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:30019;}i:721;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:" | ";}i:2;i:30022;}i:722;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:30025;}i:723;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:" | ";}i:2;i:30028;}i:724;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:30031;}i:725;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:" | ";}i:2;i:30034;}i:726;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:30037;}i:727;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1098:" |
| United | 5:30am-7:37am | 2hr 7 min | Nonstop | Economy | $248 |
| Delta | 1:20pm-3:36pm | 2hr 16 min | Nonstop | Economy | $248 |
| Alaska | 9:50pm-11:58pm | 2hr 8 min | Nonstop | Premium | $512 |

An employee is booking travel to San Francisco for February 20th to February 25th.

Recommend a hotel and flight that are in policy. Keep the recommendation concise, no longer than a sentence or two, but include pleasantries as though you are a friendly colleague helping me out:
```
 
</details>

This is the same approach that products like Microsoft Bing use to incorporate dynamic data. When you chat with Bing, it asks the bot to generate three search queries. Then they run three web searches and include the summarized results in the hidden context for the bot to use.

Summarizing this section, the trick to making a good experience is to change the context dynamically in response to whatever the user is trying to do.

> 🧙‍♂️ Giving a bot a fish is the most reliable way to ensure the bot gets a fish. You will get the most consistent and reliable results with this strategy. ";}i:2;i:30040;}i:728;a:3:{i:0;s:11:"strong_open";i:1;a:0:{}i:2;i:31138;}i:729;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:26:"Use this whenever you can.";}i:2;i:31140;}i:730;a:3:{i:0;s:12:"strong_close";i:1;a:0:{}i:2;i:31166;}i:731;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:140:"

#### Semantic Search

If you just need the bot to know a little more about the world, [a common approach is to perform a semantic search](";}i:2;i:31168;}i:732;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:102:"https://github.com/openai/openai-cookbook/blob/main/examples/Question_answering_using_embeddings.ipynb";i:1;N;}i:2;i:31308;}i:733;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1173:").

A semantic search is oriented around a document embedding – which you can think of as a fixed-length array[^5] of numbers, where each number represents some aspect of the document (e.g. if it’s a science document, maybe the  843rd number is large, but if it’s an art document the 1,115th number is large – this is overly simplistic, but conveys the idea).[^6]

In addition to computing an embedding for a document, you can also compute an embedding for a user query using the same function. If the user asks “Why is the sky blue?” – you compute the embedding of that question and, in theory, this embedding will be more similar to embeddings of documents that mention the sky than embeddings that don’t talk about the sky.

To find documents related to the user query, you compute the embedding and then find the top-N documents that have the most similar embedding. Then we place these documents (or summaries of these documents) in the hidden context for the bot to reference.

Notably, sometimes user queries are so short that the embedding isn’t particularly valuable. There is a clever technique described in [a paper published in December 2022](";}i:2;i:31410;}i:734;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:36:"https://arxiv.org/pdf/2212.10496.pdf";i:1;N;}i:2;i:32583;}i:735;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2033:") called a “Hypothetical Document Embedding” or HyDE. Using this technique, you ask the model to generate a hypothetical document in response to the user’s query, and then compute the embedding for this generated document. The model  fabricates a document out of thin air – but the approach works!

The HyDE technique uses more calls to the model, but for many use cases has notable boosts in results.

[^5]: Usually referred to as a vector.
[^6]: The vector features are learned automatically, and the specific values aren’t directly interpretable by a human without some effort.

### Teach a Bot to Fish

Sometimes you’ll want the bot to have the capability to perform actions on the user’s behalf, like adding a memo to a receipt or plotting a chart. Or perhaps we want it to retrieve data in more nuanced ways than semantic search would allow for, like retrieving the past 90 days of expenses.

In these scenarios, we need to teach the bot how to fish.

#### Command Grammars

We can give the bot a list of commands for our system to interpret, along with descriptions and examples for the commands, and then have it produce programs composed of those commands.

There are many caveats to consider when going with this approach. With complex command grammars, the bot will tend to hallucinate commands or arguments that could plausibly exist, but don’t actually. The art to getting this right is enumerating commands that have relatively high levels of abstraction, while giving the bot sufficient flexibility to compose them in novel and useful ways.

For example, giving the bot a `plot-the-last-90-days-of-expenses` command is not particularly flexible or composable in what the bot can do with it. Similarly, a `draw-pixel-at-x-y [x] [y] [rgb]` command would be far too low-level. But giving the bot a `plot-expenses` and `list-expenses` command provides some good primitives that the bot has some flexibility with.

In an example below, we use this list of commands:

| Command | Arguments | Description |
| ";}i:2;i:32619;}i:736;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:34652;}i:737;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:" | ";}i:2;i:34655;}i:738;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:34658;}i:739;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:" | ";}i:2;i:34661;}i:740;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:34664;}i:741;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:675:" |
| list-expenses | budget | Returns a list of expenses for a given budget |
| converse | message | A message to show to the user |
| plot-expenses | expenses[] | Plots a list of expenses |
| get-budget-by-name | budget_name | Retrieves a budget by name |
| list-budgets | | Returns a list of budgets the user has access to |
| add-memo | inbox_item_id, memo message | Adds a memo to the provided inbox item |

We provide this table to the model in Markdown format, which the language model handles incredibly well – presumably because OpenAI trains heavily on data from GitHub.

In this example below, we ask the model to output the commands in [reverse polish notation](";}i:2;i:34667;}i:742;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:53:"https://en.wikipedia.org/wiki/Reverse_Polish_notation";i:1;N;}i:2;i:35342;}i:743;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:83:")[^7].

[^7]: The model handles the simplicity of RPN astoundingly well.

<p align=";}i:2;i:35395;}i:744;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:35478;}i:745;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"center";}i:2;i:35479;}i:746;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:35485;}i:747;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:">
  <img width=";}i:2;i:35486;}i:748;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:35501;}i:749;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"550";}i:2;i:35502;}i:750;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:35505;}i:751;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:" src=";}i:2;i:35506;}i:752;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:35511;}i:753;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/233505150-aef4409c-03ba-4669-95d7-6c48f3c2c3ea.png";i:1;N;}i:2;i:35512;}i:754;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:35610;}i:755;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:" title=";}i:2;i:35611;}i:756;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:35618;}i:757;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:69:"A bot happily generating commands to run in response to user queries.";}i:2;i:35619;}i:758;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:35688;}i:759;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:607:">
</p>

> 🧠 There are some interesting subtle things going on in that example, beyond just command generation. When we ask it to add a memo to the “shake shack” expense, the model knows that the command `add-memo` takes an expense ID. But we never tell it the expense ID, so it looks up “Shake Shack” in the table of expenses we provided it, then grabs the ID from the corresponding ID column, and then uses that as an argument to `add-memo`.

Getting command grammars working reliably in complex situations can be tricky. The best levers we have here are to provide lots of descriptions, and as ";}i:2;i:35689;}i:760;a:3:{i:0;s:11:"strong_open";i:1;a:0:{}i:2;i:36296;}i:761;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:13:"many examples";}i:2;i:36298;}i:762;a:3:{i:0;s:12:"strong_close";i:1;a:0:{}i:2;i:36311;}i:763;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:67:" of usage as we can. Large language models are [few-shot learners](";}i:2;i:36313;}i:764;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:48:"https://en.wikipedia.org/wiki/Few-shot_learning_";i:1;N;}i:2;i:36380;}i:765;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:403:"(natural_language_processing)), meaning that they can learn a new task by being provided just a few examples. In general, the more examples you provide the better off you’ll be – but that also eats into your token budget, so it’s a balance.

Here’s a more complex example, with the output specified in JSON instead of RPN. And we use Typescript to define the return types of commands.

<p align=";}i:2;i:36428;}i:766;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:36831;}i:767;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"center";}i:2;i:36832;}i:768;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:36838;}i:769;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:">
  <img width=";}i:2;i:36839;}i:770;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:36854;}i:771;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"550";}i:2;i:36855;}i:772;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:36858;}i:773;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:" src=";}i:2;i:36859;}i:774;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:36864;}i:775;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/233505696-fc440931-9baf-4d06-80e7-54801532d63f.png";i:1;N;}i:2;i:36865;}i:776;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:36963;}i:777;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:" title=";}i:2;i:36964;}i:778;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:36971;}i:779;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:69:"A bot happily generating commands to run in response to user queries.";}i:2;i:36972;}i:780;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:37041;}i:781;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:904:">
</p>

<details>

  <summary>(Full prompt)</summary>
  
~~~
You are a financial assistant working at Brex, but you are also an expert programmer.

I am a customer of Brex.

You are to answer my questions by composing a series of commands.

The output types are:

```typescript
type LinkedAccount = {
    id: string,
    bank_details: {
        name: string,
        type: string,
    },
    brex_account_id: string,
    last_four: string,
    available_balance: {
        amount: number,
        as_of_date: Date,
    },
    current_balance: {
            amount: number,
        as_of_date: Date,
    },
}

type Expense = {
  id: string,
  memo: string,
  amount: number,
}

type Budget = {
  id: string,
  name: string,
  description: string,
  limit: {
    amount: number,
    currency: string,
  }
}
```

The commands you have available are:

| Command | Arguments | Description | Output Format |
| ";}i:2;i:37042;}i:782;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:37946;}i:783;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:" | ";}i:2;i:37949;}i:784;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:37952;}i:785;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:" | ";}i:2;i:37955;}i:786;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:37958;}i:787;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:" | ";}i:2;i:37961;}i:788;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:37964;}i:789;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:521:" |
| nth | index, values[] | Return the nth item from an array | any |
| push | value | Adds a value to the stack to be consumed by a future command | any |
| value | key, object | Returns the value associated with a key | any |
| values | key, object[] | Returns an array of values pulled from the corresponding key in array of objects | any[] |
| sum | value[] | Sums an array of numbers | number |
| plot | title, values[] | Plots the set of values in a chart with the given title | Plot |
| list-linked-accounts |  | ";}i:2;i:37967;}i:790;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:38488;}i:791;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:87:"Lists all bank connections that are eligible to make ACH transfers to Brex cash account";}i:2;i:38489;}i:792;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:38576;}i:793;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1185:" | LinkedAccount[] |
| list-expenses | budget_id | Given a budget id, returns the list of expenses for it | Expense[]
| get-budget-by-name | name | Given a name, returns the budget | Budget |
| add-memo | expense_id, message | Adds a memo to an expense | bool |
| converse | message | Send the user a message | null |

Only respond with commands.

Output the commands in JSON as an abstract syntax tree.

IMPORTANT - Only respond with a program. Do not respond with any text that isn't part of a program. Do not write prose, even if instructed. Do not explain yourself.

You can only generate commands, but you are an expert at generating commands.
~~~

</details>

This version is a bit easier to parse and interpret if your language of choice has a `JSON.parse` function.

> 🧙‍♂️ There is no industry established best format for defining a DSL for the model to generate programs. So consider this an area of active research. You will bump into limits. And as we overcome these limits, we may discover more optimal ways of defining commands.

#### ReAct

In March of 2023, Princeton and Google released a paper “[ReAct: Synergizing Reasoning and Acting in Language Models](";}i:2;i:38577;}i:794;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:36:"https://arxiv.org/pdf/2210.03629.pdf";i:1;N;}i:2;i:39762;}i:795;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:867:")”, where they introduce a variant of command grammars that allows for fully autonomous interactive execution of actions and retrieval of data.

The model is instructed to return a `thought` and an `action` that it would like to perform. Another agent (e.g. our client) then performs the `action` and returns it to the model as an `observation`. The model will then loop to return more thoughts and actions until it returns an `answer`.

This is an incredibly powerful technique, effectively allowing the bot to be its own research assistant and possibly take actions on behalf of the user. Combined with a powerful command grammar, the bot should rapidly be able to answer a massive set of user requests.

In this example, we give the model a small set of commands related to getting employee data and searching wikipedia:

| Command | Arguments | Description |
| ";}i:2;i:39798;}i:796;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:40665;}i:797;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:" | ";}i:2;i:40668;}i:798;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:40671;}i:799;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:" | ";}i:2;i:40674;}i:800;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:40677;}i:801;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:972:" |
| find_employee | name | Retrieves an employee by name |
| get_employee | id | Retrieves an employee by ID |
| get_location | id | Retrieves a location by ID |
| get_reports | employee_id | Retrieves a list of employee ids that report to the employee associated with employee_id. |
| wikipedia | article | Retrieves a wikipedia article on a topic. |

We then ask the bot a simple question, “Is my manager famous?”.

We see that the bot:

1. First looks up our employee profile.
2. From our profile, gets our manager’s id and looks up their profile.
3. Extracts our manager’s name and searches for them on Wikipedia.
    - I chose a fictional character for the manager in this scenario.
4. The bot reads the wikipedia article and concludes that can’t be my manager since it is a fictional character.
5. The bot then modifies its search to include (real person).
6. Seeing that there are no results, the bot concludes that my manager is not famous.

| ![image](";}i:2;i:40680;}i:802;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/233506839-5c8b2d77-1d78-464d-bc33-a725e12f2624.png";i:1;N;}i:2;i:41652;}i:803;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:13:") | ![image](";}i:2;i:41750;}i:804;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/233506870-05fc415d-efa2-48b7-aad9-b5035e535e6d.png";i:1;N;}i:2;i:41763;}i:805;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:") |
| ";}i:2;i:41861;}i:806;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:41867;}i:807;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:" | ";}i:2;i:41870;}i:808;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:41873;}i:809;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:360:" |

<details>
<summary>(Full prompt)</summary>

~~~
You are a helpful assistant. You run in a loop, seeking additional information to answer a user's question until you are able to answer the question.

Today is June 1, 2025. My name is Fabian Seacaster. My employee ID is 82442.

The commands to seek information are:

| Command | Arguments | Description |
| ";}i:2;i:41876;}i:810;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:42236;}i:811;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:" | ";}i:2;i:42239;}i:812;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:42242;}i:813;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:" | ";}i:2;i:42245;}i:814;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:42248;}i:815;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:405:" |
| find_employee | name | Retrieves an employee by name |
| get_employee | id | Retrieves an employee by ID |
| get_location | id | Retrieves a location by ID |
| get_reports | employee_id | Retrieves a list of employee ids that report to the employee associated with `employee_id`. |
| wikipedia | article | Retrieves a wikipedia article on a topic. |

Your response will be in JSON and will include a ";}i:2;i:42251;}i:816;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:42656;}i:817;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"Thought";}i:2;i:42657;}i:818;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:42664;}i:819;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:" + ";}i:2;i:42665;}i:820;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:42668;}i:821;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"Action";}i:2;i:42669;}i:822;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:42675;}i:823;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:88:" to retrieve data that you need in order to answer the question, or it will include the ";}i:2;i:42676;}i:824;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:42764;}i:825;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"Answer";}i:2;i:42765;}i:826;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:42771;}i:827;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:58:". When data has been retrieved, it will be included as an ";}i:2;i:42772;}i:828;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:42830;}i:829;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:11:"Observation";}i:2;i:42831;}i:830;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:42842;}i:831;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:187:".

You will continue generating thoughts and actions until you get to an answer, or conclude that you can't.

Example 1:
```
User: What is the population of Philadelphia?

Assistant: {
  ";}i:2;i:42843;}i:832;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:43030;}i:833;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"thought";}i:2;i:43031;}i:834;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:43038;}i:835;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:43039;}i:836;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:43041;}i:837;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:56:"Wikipedia likely has this information. I'm looking it up";}i:2;i:43042;}i:838;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"...";}i:2;i:43098;}i:839;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:43101;}i:840;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:4:",
  ";}i:2;i:43102;}i:841;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:43106;}i:842;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"action";}i:2;i:43107;}i:843;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:43113;}i:844;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:": {";}i:2;i:43114;}i:845;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:43117;}i:846;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"command";}i:2;i:43118;}i:847;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:43125;}i:848;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:43126;}i:849;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:43128;}i:850;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:9:"wikipedia";}i:2;i:43129;}i:851;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:43138;}i:852;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:", ";}i:2;i:43139;}i:853;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:43141;}i:854;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"article";}i:2;i:43142;}i:855;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:43149;}i:856;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:43150;}i:857;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:43152;}i:858;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:"Philadelphia";}i:2;i:43153;}i:859;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:43165;}i:860;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:20:"}
}

Assistant: {
  ";}i:2;i:43166;}i:861;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:43186;}i:862;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:11:"observation";}i:2;i:43187;}i:863;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:43198;}i:864;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:43199;}i:865;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:43201;}i:866;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:862:"Philadelphia, often called Philly, is the largest city in the Commonwealth of Pennsylvania and the second-largest city in both the Northeast megalopolis and Mid-Atlantic regions after New York City. It is one of the most historically significant cities in the United States and served as the nation's capital city until 1800. Philadelphia is the nation's sixth-largest city with a population of 1,603,797 as of the 2020 census. Since 1854, the city has been coextensive with Philadelphia County, the most populous county in Pennsylvania and the urban core of the Delaware Valley, the nation's seventh-largest and one of the world's largest metropolitan regions with 6.245 million residents in 2020. Philadelphia is known for its extensive contributions to American history and for its role in the life sciences, business and industry, art, literature, and music.";}i:2;i:43202;}i:867;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:44064;}i:868;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:19:"
}

Assistant: {
  ";}i:2;i:44065;}i:869;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:44084;}i:870;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"answer";}i:2;i:44085;}i:871;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:44091;}i:872;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:44092;}i:873;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:44094;}i:874;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:75:"The population of Philadelphia, as of the 2020 census, is 1,603,797 people.";}i:2;i:44095;}i:875;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:44170;}i:876;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:72:"
}
```

Example 2:
```
User: What is my manager's city?

Assistant: {
  ";}i:2;i:44171;}i:877;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:44243;}i:878;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"thought";}i:2;i:44244;}i:879;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:44251;}i:880;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:44252;}i:881;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:44254;}i:882;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:74:"Let me look up who your manager is. First, I need to look up your profile.";}i:2;i:44255;}i:883;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:44329;}i:884;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:4:",
  ";}i:2;i:44330;}i:885;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:44334;}i:886;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"action";}i:2;i:44335;}i:887;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:44341;}i:888;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:": {";}i:2;i:44342;}i:889;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:44345;}i:890;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"command";}i:2;i:44346;}i:891;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:44353;}i:892;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:44354;}i:893;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:44356;}i:894;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:"get_employee";}i:2;i:44357;}i:895;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:44369;}i:896;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:", ";}i:2;i:44370;}i:897;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:44372;}i:898;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:"id";}i:2;i:44373;}i:899;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:44375;}i:900;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:27:": 92352}
}

Assistant: {
  ";}i:2;i:44376;}i:901;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:44403;}i:902;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:11:"observation";}i:2;i:44404;}i:903;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:44415;}i:904;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:8:": {
    ";}i:2;i:44416;}i:905;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:44424;}i:906;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:"id";}i:2;i:44425;}i:907;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:44427;}i:908;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:13:": 78334,
    ";}i:2;i:44428;}i:909;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:44441;}i:910;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:4:"name";}i:2;i:44442;}i:911;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:44446;}i:912;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:44447;}i:913;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:44449;}i:914;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:11:"Ms. Manager";}i:2;i:44450;}i:915;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:44461;}i:916;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:",
    ";}i:2;i:44462;}i:917;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:44468;}i:918;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:11:"location_id";}i:2;i:44469;}i:919;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:44480;}i:920;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:29:": 8832
  }
}

Assistant: {
  ";}i:2;i:44481;}i:921;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:44510;}i:922;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"thought";}i:2;i:44511;}i:923;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:44518;}i:924;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:44519;}i:925;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:44521;}i:926;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:59:"Your manager is Ms. Manager. I'm looking up their location.";}i:2;i:44522;}i:927;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:44581;}i:928;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:4:",
  ";}i:2;i:44582;}i:929;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:44586;}i:930;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"action";}i:2;i:44587;}i:931;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:44593;}i:932;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:": {";}i:2;i:44594;}i:933;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:44597;}i:934;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"command";}i:2;i:44598;}i:935;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:44605;}i:936;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:44606;}i:937;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:44608;}i:938;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:"get_location";}i:2;i:44609;}i:939;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:44621;}i:940;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:", ";}i:2;i:44622;}i:941;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:44624;}i:942;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:"id";}i:2;i:44625;}i:943;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:44627;}i:944;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:26:": 8832}
}

Assistant: {
  ";}i:2;i:44628;}i:945;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:44654;}i:946;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:11:"observation";}i:2;i:44655;}i:947;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:44666;}i:948;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:8:": {
    ";}i:2;i:44667;}i:949;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:44675;}i:950;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:"id";}i:2;i:44676;}i:951;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:44678;}i:952;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:": 8832,
    ";}i:2;i:44679;}i:953;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:44691;}i:954;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:4:"name";}i:2;i:44692;}i:955;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:44696;}i:956;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:44697;}i:957;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:44699;}i:958;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:"Philadelphia";}i:2;i:44700;}i:959;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:44712;}i:960;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:23:"
  }
}

Assistant: {
  ";}i:2;i:44713;}i:961;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:44736;}i:962;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"answer";}i:2;i:44737;}i:963;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:44743;}i:964;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:44744;}i:965;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:44746;}i:966;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:35:"Your manager lives in Philadelphia.";}i:2;i:44747;}i:967;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:44782;}i:968;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1489:"
}
```
~~~
</details>

#### GPT-4 vs GPT-3.5

In most of the examples in this doc, the difference between GPT-3.5 and GPT-4 is negligible, but for “teaching a bot to fish” scenarios the difference between the models is notable.

None of the above examples of command grammars, for example, work without meaningful modifications for GPT-3.5. At a minimum, you have to provide a number of examples (at least one usage example per command) before you get any reasonable results. And, for complex sets of commands, it may hallucinate new commands or create fictional arguments.

With a sufficiently thorough hidden prompt, you should be able to overcome these limitations. GPT-4 is capable of far more consistent and complex logic with far simpler prompts (and can get by with zero or  small numbers of examples – though it is always beneficial to include as many as possible).

## Strategies

This section contains examples and strategies for specific needs or problems. For successful prompt engineering, you will need to combine some subset of all of the strategies enumerated in this document. Don’t be afraid to mix and match things – or invent your own approaches.

### Embedding Data

In hidden contexts, you’ll frequently want to embed all sorts of data. The specific strategy will vary depending on the type and quantity of data you are embedding.

#### Simple Lists

For one-off objects, enumerating fields + values in a normal bulleted list works pretty well:

<p align=";}i:2;i:44783;}i:969;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:46272;}i:970;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"center";}i:2;i:46273;}i:971;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:46279;}i:972;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:">
  <img width=";}i:2;i:46280;}i:973;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:46295;}i:974;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"550";}i:2;i:46296;}i:975;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:46299;}i:976;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:" src=";}i:2;i:46300;}i:977;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:46305;}i:978;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/233507156-0bdbc0af-d977-44e0-a8d5-b30538c5bbd9.png";i:1;N;}i:2;i:46306;}i:979;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:46404;}i:980;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:" title=";}i:2;i:46405;}i:981;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:46412;}i:982;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:61:"GPT-4 extracting Steve’s occupation from a list attributes.";}i:2;i:46413;}i:983;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:46474;}i:984;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:174:">
</p>

It will also work for larger sets of things, but there are other formats for lists of data that GPT handles more reliably. Regardless, here’s an example:

<p align=";}i:2;i:46475;}i:985;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:46649;}i:986;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"center";}i:2;i:46650;}i:987;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:46656;}i:988;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:">
  <img width=";}i:2;i:46657;}i:989;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:46672;}i:990;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"550";}i:2;i:46673;}i:991;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:46676;}i:992;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:" src=";}i:2;i:46677;}i:993;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:46682;}i:994;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/233507223-9cda591e-62f3-4339-b227-a07c37b90724.png";i:1;N;}i:2;i:46683;}i:995;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:46781;}i:996;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:" title=";}i:2;i:46782;}i:997;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:46789;}i:998;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:50:"GPT-4 answering questions about a set of expenses.";}i:2;i:46790;}i:999;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:46840;}i:1000;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:345:">
</p>

#### Markdown Tables

Markdown tables are great for scenarios where you have many items of the same type to enumerate.

Fortunately, OpenAI’s models are exceptionally good at working with Markdown tables (presumably from the tons of GitHub data they’ve trained on).

We can reframe the above using Markdown tables instead:

<p align=";}i:2;i:46841;}i:1001;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:47186;}i:1002;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"center";}i:2;i:47187;}i:1003;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:47193;}i:1004;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:">
  <img width=";}i:2;i:47194;}i:1005;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:47209;}i:1006;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"550";}i:2;i:47210;}i:1007;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:47213;}i:1008;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:" src=";}i:2;i:47214;}i:1009;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:47219;}i:1010;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/233507313-7ccd825c-71b9-46d3-80c9-30bf97a8e090.png";i:1;N;}i:2;i:47220;}i:1011;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:47318;}i:1012;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:" title=";}i:2;i:47319;}i:1013;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:47326;}i:1014;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:72:"GPT-4 answering questions about a set of expenses from a Markdown table.";}i:2;i:47327;}i:1015;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:47399;}i:1016;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:17:">
</p>

<p align=";}i:2;i:47400;}i:1017;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:47417;}i:1018;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"center";}i:2;i:47418;}i:1019;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:47424;}i:1020;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:">
  <img width=";}i:2;i:47425;}i:1021;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:47440;}i:1022;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"550";}i:2;i:47441;}i:1023;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:47444;}i:1024;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:" src=";}i:2;i:47445;}i:1025;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:47450;}i:1026;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/233507395-b8ecb641-726c-4e57-b85e-13f6b7717f22.png";i:1;N;}i:2;i:47451;}i:1027;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:47549;}i:1028;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:" title=";}i:2;i:47550;}i:1029;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:47557;}i:1030;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:72:"GPT-4 answering questions about a set of expenses from a Markdown table.";}i:2;i:47558;}i:1031;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:47630;}i:1032;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:999:">
</p>

> 🧠 Note that in this last example, the items in the table have an explicit date, February 2nd. In our question, we asked about “today”. And earlier in the prompt we mentioned that today was Feb 2. The model correctly handled the transitive inference – converting “today” to “February 2nd” and then looking up “February 2nd” in the table.

#### JSON

Markdown tables work really well for many use cases and should be preferred due to their density and ability for the model to handle them reliably, but you may run into scenarios where you have many columns and the model struggles with it or every item has some custom attributes and it doesn’t make sense to have dozens of columns of empty data.

In these scenarios, JSON is another format that the model handles really well. The close proximity of `keys` to their `values` makes it easy for the model to keep the mapping straight.

Here is the same example from the Markdown table, but with JSON instead:

<p align=";}i:2;i:47631;}i:1033;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:48630;}i:1034;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"center";}i:2;i:48631;}i:1035;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:48637;}i:1036;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:">
  <img width=";}i:2;i:48638;}i:1037;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:48653;}i:1038;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"550";}i:2;i:48654;}i:1039;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:48657;}i:1040;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:" src=";}i:2;i:48658;}i:1041;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:48663;}i:1042;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/233507559-26e6615d-4896-4a2c-b6ff-44cbd7d349dc.png";i:1;N;}i:2;i:48664;}i:1043;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:48762;}i:1044;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:" title=";}i:2;i:48763;}i:1045;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:48770;}i:1046;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:67:"GPT-4 answering questions about a set of expenses from a JSON blob.";}i:2;i:48771;}i:1047;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:48838;}i:1048;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:311:">
</p>

#### Freeform Text

Occasionally you’ll want to include freeform text in a prompt that you would like to delineate from the rest of the prompt – such as embedding a document for the bot to reference. In these scenarios, surrounding the document with triple backticks, ```, works well[^8].

<p align=";}i:2;i:48839;}i:1049;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:49150;}i:1050;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"center";}i:2;i:49151;}i:1051;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:49157;}i:1052;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:">
  <img width=";}i:2;i:49158;}i:1053;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:49173;}i:1054;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"550";}i:2;i:49174;}i:1055;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:49177;}i:1056;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:" src=";}i:2;i:49178;}i:1057;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:49183;}i:1058;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/233507684-93222728-e216-47b4-8554-04acf9ec6201.png";i:1;N;}i:2;i:49184;}i:1059;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:49282;}i:1060;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:" title=";}i:2;i:49283;}i:1061;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:49290;}i:1062;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:67:"GPT-4 answering questions about a set of expenses from a JSON blob.";}i:2;i:49291;}i:1063;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:49358;}i:1064;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:328:">
</p>

[^8]: A good rule of thumb for anything you’re doing in prompts is to lean heavily on things the model would have learned from GitHub.

#### Nested Data

Not all data is flat and linear. Sometimes you’ll need to embed data that is nested or has relations to other data. In these scenarios, lean on `JSON`:

<p align=";}i:2;i:49359;}i:1065;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:49687;}i:1066;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"center";}i:2;i:49688;}i:1067;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:49694;}i:1068;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:">
  <img width=";}i:2;i:49695;}i:1069;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:49710;}i:1070;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"550";}i:2;i:49711;}i:1071;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:49714;}i:1072;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:" src=";}i:2;i:49715;}i:1073;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:49720;}i:1074;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/233507758-7baffcaa-647b-4869-9cfb-a7cf8849c453.png";i:1;N;}i:2;i:49721;}i:1075;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:49819;}i:1076;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:" title=";}i:2;i:49820;}i:1077;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:49827;}i:1078;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:40:"GPT-4 handles nested JSON very reliably.";}i:2;i:49828;}i:1079;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:49868;}i:1080;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:158:">
</p>

<details>
<summary>(Full prompt)</summary>

~~~
You are a helpful assistant. You answer questions about users. Here is what you know about them:

{
  ";}i:2;i:49869;}i:1081;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50027;}i:1082;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"users";}i:2;i:50028;}i:1083;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50033;}i:1084;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:16:": [
    {
      ";}i:2;i:50034;}i:1085;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50050;}i:1086;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:"id";}i:2;i:50051;}i:1087;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50053;}i:1088;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:11:": 1,
      ";}i:2;i:50054;}i:1089;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50065;}i:1090;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:4:"name";}i:2;i:50066;}i:1091;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50070;}i:1092;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:50071;}i:1093;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50073;}i:1094;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:8:"John Doe";}i:2;i:50074;}i:1095;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50082;}i:1096;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:8:",
      ";}i:2;i:50083;}i:1097;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50091;}i:1098;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"contact";}i:2;i:50092;}i:1099;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50099;}i:1100;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:": {
        ";}i:2;i:50100;}i:1101;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50112;}i:1102;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"address";}i:2;i:50113;}i:1103;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50120;}i:1104;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:14:": {
          ";}i:2;i:50121;}i:1105;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50135;}i:1106;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"street";}i:2;i:50136;}i:1107;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50142;}i:1108;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:50143;}i:1109;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50145;}i:1110;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:11:"123 Main St";}i:2;i:50146;}i:1111;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50157;}i:1112;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:",
          ";}i:2;i:50158;}i:1113;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50170;}i:1114;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:4:"city";}i:2;i:50171;}i:1115;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50175;}i:1116;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:50176;}i:1117;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50178;}i:1118;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"Anytown";}i:2;i:50179;}i:1119;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50186;}i:1120;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:",
          ";}i:2;i:50187;}i:1121;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50199;}i:1122;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"state";}i:2;i:50200;}i:1123;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50205;}i:1124;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:50206;}i:1125;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50208;}i:1126;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:"CA";}i:2;i:50209;}i:1127;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50211;}i:1128;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:",
          ";}i:2;i:50212;}i:1129;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50224;}i:1130;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"zip";}i:2;i:50225;}i:1131;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50228;}i:1132;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:50229;}i:1133;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50231;}i:1134;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"12345";}i:2;i:50232;}i:1135;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50237;}i:1136;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:20:"
        },
        ";}i:2;i:50238;}i:1137;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50258;}i:1138;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"phone";}i:2;i:50259;}i:1139;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50264;}i:1140;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:50265;}i:1141;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50267;}i:1142;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:"555-555-1234";}i:2;i:50268;}i:1143;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50280;}i:1144;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:10:",
        ";}i:2;i:50281;}i:1145;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50291;}i:1146;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"email";}i:2;i:50292;}i:1147;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50297;}i:1148;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:50298;}i:1149;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50300;}i:1150;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:19:"johndoe@example.com";}i:2;i:50301;}i:1151;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50320;}i:1152;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:28:"
      }
    },
    {
      ";}i:2;i:50321;}i:1153;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50349;}i:1154;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:"id";}i:2;i:50350;}i:1155;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50352;}i:1156;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:11:": 2,
      ";}i:2;i:50353;}i:1157;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50364;}i:1158;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:4:"name";}i:2;i:50365;}i:1159;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50369;}i:1160;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:50370;}i:1161;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50372;}i:1162;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:10:"Jane Smith";}i:2;i:50373;}i:1163;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50383;}i:1164;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:8:",
      ";}i:2;i:50384;}i:1165;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50392;}i:1166;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"contact";}i:2;i:50393;}i:1167;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50400;}i:1168;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:": {
        ";}i:2;i:50401;}i:1169;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50413;}i:1170;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"address";}i:2;i:50414;}i:1171;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50421;}i:1172;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:14:": {
          ";}i:2;i:50422;}i:1173;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50436;}i:1174;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"street";}i:2;i:50437;}i:1175;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50443;}i:1176;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:50444;}i:1177;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50446;}i:1178;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:10:"456 Elm St";}i:2;i:50447;}i:1179;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50457;}i:1180;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:",
          ";}i:2;i:50458;}i:1181;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50470;}i:1182;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:4:"city";}i:2;i:50471;}i:1183;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50475;}i:1184;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:50476;}i:1185;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50478;}i:1186;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:8:"Sometown";}i:2;i:50479;}i:1187;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50487;}i:1188;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:",
          ";}i:2;i:50488;}i:1189;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50500;}i:1190;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"state";}i:2;i:50501;}i:1191;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50506;}i:1192;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:50507;}i:1193;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50509;}i:1194;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:"TX";}i:2;i:50510;}i:1195;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50512;}i:1196;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:",
          ";}i:2;i:50513;}i:1197;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50525;}i:1198;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"zip";}i:2;i:50526;}i:1199;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50529;}i:1200;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:50530;}i:1201;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50532;}i:1202;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"54321";}i:2;i:50533;}i:1203;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50538;}i:1204;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:20:"
        },
        ";}i:2;i:50539;}i:1205;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50559;}i:1206;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"phone";}i:2;i:50560;}i:1207;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50565;}i:1208;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:50566;}i:1209;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50568;}i:1210;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:"555-555-5678";}i:2;i:50569;}i:1211;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50581;}i:1212;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:10:",
        ";}i:2;i:50582;}i:1213;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50592;}i:1214;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"email";}i:2;i:50593;}i:1215;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50598;}i:1216;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:50599;}i:1217;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50601;}i:1218;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:21:"janesmith@example.com";}i:2;i:50602;}i:1219;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50623;}i:1220;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:28:"
      }
    },
    {
      ";}i:2;i:50624;}i:1221;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50652;}i:1222;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:"id";}i:2;i:50653;}i:1223;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50655;}i:1224;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:11:": 3,
      ";}i:2;i:50656;}i:1225;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50667;}i:1226;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:4:"name";}i:2;i:50668;}i:1227;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50672;}i:1228;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:50673;}i:1229;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50675;}i:1230;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:13:"Alice Johnson";}i:2;i:50676;}i:1231;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50689;}i:1232;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:8:",
      ";}i:2;i:50690;}i:1233;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50698;}i:1234;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"contact";}i:2;i:50699;}i:1235;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50706;}i:1236;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:": {
        ";}i:2;i:50707;}i:1237;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50719;}i:1238;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"address";}i:2;i:50720;}i:1239;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50727;}i:1240;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:14:": {
          ";}i:2;i:50728;}i:1241;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50742;}i:1242;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"street";}i:2;i:50743;}i:1243;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50749;}i:1244;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:50750;}i:1245;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50752;}i:1246;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:10:"789 Oak St";}i:2;i:50753;}i:1247;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50763;}i:1248;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:",
          ";}i:2;i:50764;}i:1249;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50776;}i:1250;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:4:"city";}i:2;i:50777;}i:1251;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50781;}i:1252;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:50782;}i:1253;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50784;}i:1254;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:9:"Othertown";}i:2;i:50785;}i:1255;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50794;}i:1256;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:",
          ";}i:2;i:50795;}i:1257;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50807;}i:1258;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"state";}i:2;i:50808;}i:1259;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50813;}i:1260;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:50814;}i:1261;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50816;}i:1262;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:"NY";}i:2;i:50817;}i:1263;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50819;}i:1264;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:",
          ";}i:2;i:50820;}i:1265;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50832;}i:1266;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"zip";}i:2;i:50833;}i:1267;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50836;}i:1268;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:50837;}i:1269;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50839;}i:1270;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"67890";}i:2;i:50840;}i:1271;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50845;}i:1272;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:20:"
        },
        ";}i:2;i:50846;}i:1273;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50866;}i:1274;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"phone";}i:2;i:50867;}i:1275;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50872;}i:1276;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:50873;}i:1277;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50875;}i:1278;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:"555-555-2468";}i:2;i:50876;}i:1279;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50888;}i:1280;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:10:",
        ";}i:2;i:50889;}i:1281;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50899;}i:1282;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"email";}i:2;i:50900;}i:1283;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50905;}i:1284;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:50906;}i:1285;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50908;}i:1286;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:24:"alicejohnson@example.com";}i:2;i:50909;}i:1287;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50933;}i:1288;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:28:"
      }
    },
    {
      ";}i:2;i:50934;}i:1289;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50962;}i:1290;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:"id";}i:2;i:50963;}i:1291;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50965;}i:1292;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:11:": 4,
      ";}i:2;i:50966;}i:1293;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50977;}i:1294;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:4:"name";}i:2;i:50978;}i:1295;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50982;}i:1296;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:50983;}i:1297;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:50985;}i:1298;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:"Bob Williams";}i:2;i:50986;}i:1299;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:50998;}i:1300;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:8:",
      ";}i:2;i:50999;}i:1301;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51007;}i:1302;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"contact";}i:2;i:51008;}i:1303;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51015;}i:1304;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:": {
        ";}i:2;i:51016;}i:1305;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51028;}i:1306;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"address";}i:2;i:51029;}i:1307;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51036;}i:1308;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:14:": {
          ";}i:2;i:51037;}i:1309;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51051;}i:1310;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"street";}i:2;i:51052;}i:1311;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51058;}i:1312;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:51059;}i:1313;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51061;}i:1314;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:"135 Maple St";}i:2;i:51062;}i:1315;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51074;}i:1316;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:",
          ";}i:2;i:51075;}i:1317;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51087;}i:1318;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:4:"city";}i:2;i:51088;}i:1319;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51092;}i:1320;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:51093;}i:1321;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51095;}i:1322;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:8:"Thistown";}i:2;i:51096;}i:1323;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51104;}i:1324;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:",
          ";}i:2;i:51105;}i:1325;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51117;}i:1326;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"state";}i:2;i:51118;}i:1327;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51123;}i:1328;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:51124;}i:1329;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51126;}i:1330;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:"FL";}i:2;i:51127;}i:1331;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51129;}i:1332;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:",
          ";}i:2;i:51130;}i:1333;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51142;}i:1334;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"zip";}i:2;i:51143;}i:1335;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51146;}i:1336;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:51147;}i:1337;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51149;}i:1338;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"98765";}i:2;i:51150;}i:1339;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51155;}i:1340;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:20:"
        },
        ";}i:2;i:51156;}i:1341;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51176;}i:1342;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"phone";}i:2;i:51177;}i:1343;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51182;}i:1344;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:51183;}i:1345;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51185;}i:1346;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:"555-555-8642";}i:2;i:51186;}i:1347;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51198;}i:1348;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:10:",
        ";}i:2;i:51199;}i:1349;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51209;}i:1350;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"email";}i:2;i:51210;}i:1351;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51215;}i:1352;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:51216;}i:1353;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51218;}i:1354;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:23:"bobwilliams@example.com";}i:2;i:51219;}i:1355;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51242;}i:1356;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:28:"
      }
    },
    {
      ";}i:2;i:51243;}i:1357;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51271;}i:1358;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:"id";}i:2;i:51272;}i:1359;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51274;}i:1360;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:11:": 5,
      ";}i:2;i:51275;}i:1361;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51286;}i:1362;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:4:"name";}i:2;i:51287;}i:1363;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51291;}i:1364;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:51292;}i:1365;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51294;}i:1366;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:13:"Charlie Brown";}i:2;i:51295;}i:1367;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51308;}i:1368;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:8:",
      ";}i:2;i:51309;}i:1369;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51317;}i:1370;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"contact";}i:2;i:51318;}i:1371;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51325;}i:1372;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:": {
        ";}i:2;i:51326;}i:1373;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51338;}i:1374;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"address";}i:2;i:51339;}i:1375;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51346;}i:1376;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:14:": {
          ";}i:2;i:51347;}i:1377;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51361;}i:1378;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"street";}i:2;i:51362;}i:1379;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51368;}i:1380;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:51369;}i:1381;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51371;}i:1382;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:11:"246 Pine St";}i:2;i:51372;}i:1383;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51383;}i:1384;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:",
          ";}i:2;i:51384;}i:1385;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51396;}i:1386;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:4:"city";}i:2;i:51397;}i:1387;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51401;}i:1388;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:51402;}i:1389;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51404;}i:1390;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:9:"Thatstown";}i:2;i:51405;}i:1391;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51414;}i:1392;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:",
          ";}i:2;i:51415;}i:1393;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51427;}i:1394;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"state";}i:2;i:51428;}i:1395;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51433;}i:1396;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:51434;}i:1397;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51436;}i:1398;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:"WA";}i:2;i:51437;}i:1399;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51439;}i:1400;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:",
          ";}i:2;i:51440;}i:1401;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51452;}i:1402;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"zip";}i:2;i:51453;}i:1403;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51456;}i:1404;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:51457;}i:1405;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51459;}i:1406;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"86420";}i:2;i:51460;}i:1407;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51465;}i:1408;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:20:"
        },
        ";}i:2;i:51466;}i:1409;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51486;}i:1410;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"phone";}i:2;i:51487;}i:1411;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51492;}i:1412;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:51493;}i:1413;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51495;}i:1414;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:"555-555-7531";}i:2;i:51496;}i:1415;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51508;}i:1416;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:10:",
        ";}i:2;i:51509;}i:1417;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51519;}i:1418;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"email";}i:2;i:51520;}i:1419;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51525;}i:1420;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:51526;}i:1421;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51528;}i:1422;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:24:"charliebrown@example.com";}i:2;i:51529;}i:1423;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51553;}i:1424;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:28:"
      }
    },
    {
      ";}i:2;i:51554;}i:1425;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51582;}i:1426;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:"id";}i:2;i:51583;}i:1427;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51585;}i:1428;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:11:": 6,
      ";}i:2;i:51586;}i:1429;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51597;}i:1430;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:4:"name";}i:2;i:51598;}i:1431;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51602;}i:1432;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:51603;}i:1433;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51605;}i:1434;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:11:"Diane Davis";}i:2;i:51606;}i:1435;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51617;}i:1436;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:8:",
      ";}i:2;i:51618;}i:1437;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51626;}i:1438;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"contact";}i:2;i:51627;}i:1439;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51634;}i:1440;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:": {
        ";}i:2;i:51635;}i:1441;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51647;}i:1442;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"address";}i:2;i:51648;}i:1443;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51655;}i:1444;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:14:": {
          ";}i:2;i:51656;}i:1445;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51670;}i:1446;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"street";}i:2;i:51671;}i:1447;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51677;}i:1448;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:51678;}i:1449;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51680;}i:1450;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:13:"369 Willow St";}i:2;i:51681;}i:1451;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51694;}i:1452;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:",
          ";}i:2;i:51695;}i:1453;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51707;}i:1454;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:4:"city";}i:2;i:51708;}i:1455;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51712;}i:1456;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:51713;}i:1457;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51715;}i:1458;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"Sumtown";}i:2;i:51716;}i:1459;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51723;}i:1460;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:",
          ";}i:2;i:51724;}i:1461;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51736;}i:1462;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"state";}i:2;i:51737;}i:1463;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51742;}i:1464;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:51743;}i:1465;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51745;}i:1466;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:"CO";}i:2;i:51746;}i:1467;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51748;}i:1468;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:",
          ";}i:2;i:51749;}i:1469;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51761;}i:1470;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"zip";}i:2;i:51762;}i:1471;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51765;}i:1472;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:51766;}i:1473;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51768;}i:1474;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"15980";}i:2;i:51769;}i:1475;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51774;}i:1476;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:20:"
        },
        ";}i:2;i:51775;}i:1477;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51795;}i:1478;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"phone";}i:2;i:51796;}i:1479;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51801;}i:1480;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:51802;}i:1481;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51804;}i:1482;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:"555-555-9512";}i:2;i:51805;}i:1483;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51817;}i:1484;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:10:",
        ";}i:2;i:51818;}i:1485;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51828;}i:1486;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"email";}i:2;i:51829;}i:1487;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51834;}i:1488;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:51835;}i:1489;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51837;}i:1490;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:22:"dianedavis@example.com";}i:2;i:51838;}i:1491;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51860;}i:1492;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:28:"
      }
    },
    {
      ";}i:2;i:51861;}i:1493;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51889;}i:1494;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:"id";}i:2;i:51890;}i:1495;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51892;}i:1496;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:11:": 7,
      ";}i:2;i:51893;}i:1497;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51904;}i:1498;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:4:"name";}i:2;i:51905;}i:1499;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51909;}i:1500;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:51910;}i:1501;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51912;}i:1502;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:"Edward Martinez";}i:2;i:51913;}i:1503;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51928;}i:1504;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:8:",
      ";}i:2;i:51929;}i:1505;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51937;}i:1506;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"contact";}i:2;i:51938;}i:1507;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51945;}i:1508;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:": {
        ";}i:2;i:51946;}i:1509;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51958;}i:1510;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"address";}i:2;i:51959;}i:1511;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51966;}i:1512;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:14:": {
          ";}i:2;i:51967;}i:1513;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51981;}i:1514;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"street";}i:2;i:51982;}i:1515;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:51988;}i:1516;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:51989;}i:1517;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:51991;}i:1518;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:"482 Aspen St";}i:2;i:51992;}i:1519;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52004;}i:1520;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:",
          ";}i:2;i:52005;}i:1521;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52017;}i:1522;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:4:"city";}i:2;i:52018;}i:1523;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52022;}i:1524;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:52023;}i:1525;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52025;}i:1526;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"Newtown";}i:2;i:52026;}i:1527;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52033;}i:1528;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:",
          ";}i:2;i:52034;}i:1529;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52046;}i:1530;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"state";}i:2;i:52047;}i:1531;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52052;}i:1532;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:52053;}i:1533;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52055;}i:1534;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:"MI";}i:2;i:52056;}i:1535;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52058;}i:1536;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:",
          ";}i:2;i:52059;}i:1537;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52071;}i:1538;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"zip";}i:2;i:52072;}i:1539;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52075;}i:1540;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:52076;}i:1541;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52078;}i:1542;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"35742";}i:2;i:52079;}i:1543;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52084;}i:1544;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:20:"
        },
        ";}i:2;i:52085;}i:1545;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52105;}i:1546;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"phone";}i:2;i:52106;}i:1547;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52111;}i:1548;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:52112;}i:1549;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52114;}i:1550;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:"555-555-6813";}i:2;i:52115;}i:1551;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52127;}i:1552;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:10:",
        ";}i:2;i:52128;}i:1553;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52138;}i:1554;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"email";}i:2;i:52139;}i:1555;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52144;}i:1556;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:52145;}i:1557;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52147;}i:1558;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:26:"edwardmartinez@example.com";}i:2;i:52148;}i:1559;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52174;}i:1560;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:28:"
      }
    },
    {
      ";}i:2;i:52175;}i:1561;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52203;}i:1562;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:"id";}i:2;i:52204;}i:1563;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52206;}i:1564;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:11:": 8,
      ";}i:2;i:52207;}i:1565;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52218;}i:1566;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:4:"name";}i:2;i:52219;}i:1567;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52223;}i:1568;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:52224;}i:1569;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52226;}i:1570;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:"Fiona Taylor";}i:2;i:52227;}i:1571;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52239;}i:1572;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:8:",
      ";}i:2;i:52240;}i:1573;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52248;}i:1574;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"contact";}i:2;i:52249;}i:1575;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52256;}i:1576;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:": {
        ";}i:2;i:52257;}i:1577;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52269;}i:1578;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"address";}i:2;i:52270;}i:1579;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52277;}i:1580;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:14:": {
          ";}i:2;i:52278;}i:1581;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52292;}i:1582;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"street";}i:2;i:52293;}i:1583;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52299;}i:1584;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:52300;}i:1585;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52302;}i:1586;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:"531 Birch St";}i:2;i:52303;}i:1587;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52315;}i:1588;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:",
          ";}i:2;i:52316;}i:1589;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52328;}i:1590;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:4:"city";}i:2;i:52329;}i:1591;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52333;}i:1592;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:52334;}i:1593;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52336;}i:1594;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"Oldtown";}i:2;i:52337;}i:1595;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52344;}i:1596;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:",
          ";}i:2;i:52345;}i:1597;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52357;}i:1598;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"state";}i:2;i:52358;}i:1599;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52363;}i:1600;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:52364;}i:1601;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52366;}i:1602;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:"OH";}i:2;i:52367;}i:1603;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52369;}i:1604;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:",
          ";}i:2;i:52370;}i:1605;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52382;}i:1606;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"zip";}i:2;i:52383;}i:1607;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52386;}i:1608;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:52387;}i:1609;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52389;}i:1610;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"85249";}i:2;i:52390;}i:1611;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52395;}i:1612;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:20:"
        },
        ";}i:2;i:52396;}i:1613;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52416;}i:1614;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"phone";}i:2;i:52417;}i:1615;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52422;}i:1616;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:52423;}i:1617;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52425;}i:1618;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:"555-555-4268";}i:2;i:52426;}i:1619;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52438;}i:1620;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:10:",
        ";}i:2;i:52439;}i:1621;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52449;}i:1622;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"email";}i:2;i:52450;}i:1623;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52455;}i:1624;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:52456;}i:1625;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52458;}i:1626;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:23:"fionataylor@example.com";}i:2;i:52459;}i:1627;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52482;}i:1628;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:28:"
      }
    },
    {
      ";}i:2;i:52483;}i:1629;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52511;}i:1630;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:"id";}i:2;i:52512;}i:1631;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52514;}i:1632;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:11:": 9,
      ";}i:2;i:52515;}i:1633;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52526;}i:1634;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:4:"name";}i:2;i:52527;}i:1635;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52531;}i:1636;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:52532;}i:1637;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52534;}i:1638;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:"George Thompson";}i:2;i:52535;}i:1639;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52550;}i:1640;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:8:",
      ";}i:2;i:52551;}i:1641;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52559;}i:1642;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"contact";}i:2;i:52560;}i:1643;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52567;}i:1644;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:": {
        ";}i:2;i:52568;}i:1645;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52580;}i:1646;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"address";}i:2;i:52581;}i:1647;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52588;}i:1648;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:14:": {
          ";}i:2;i:52589;}i:1649;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52603;}i:1650;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"street";}i:2;i:52604;}i:1651;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52610;}i:1652;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:52611;}i:1653;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52613;}i:1654;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:"678 Cedar St";}i:2;i:52614;}i:1655;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52626;}i:1656;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:",
          ";}i:2;i:52627;}i:1657;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52639;}i:1658;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:4:"city";}i:2;i:52640;}i:1659;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52644;}i:1660;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:52645;}i:1661;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52647;}i:1662;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:8:"Nexttown";}i:2;i:52648;}i:1663;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52656;}i:1664;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:",
          ";}i:2;i:52657;}i:1665;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52669;}i:1666;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"state";}i:2;i:52670;}i:1667;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52675;}i:1668;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:52676;}i:1669;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52678;}i:1670;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:"GA";}i:2;i:52679;}i:1671;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52681;}i:1672;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:",
          ";}i:2;i:52682;}i:1673;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52694;}i:1674;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"zip";}i:2;i:52695;}i:1675;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52698;}i:1676;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:52699;}i:1677;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52701;}i:1678;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"74125";}i:2;i:52702;}i:1679;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52707;}i:1680;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:20:"
        },
        ";}i:2;i:52708;}i:1681;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52728;}i:1682;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"phone";}i:2;i:52729;}i:1683;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52734;}i:1684;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:52735;}i:1685;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52737;}i:1686;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:"555-555-3142";}i:2;i:52738;}i:1687;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52750;}i:1688;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:10:",
        ";}i:2;i:52751;}i:1689;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52761;}i:1690;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"email";}i:2;i:52762;}i:1691;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52767;}i:1692;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:52768;}i:1693;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52770;}i:1694;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:26:"georgethompson@example.com";}i:2;i:52771;}i:1695;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52797;}i:1696;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:28:"
      }
    },
    {
      ";}i:2;i:52798;}i:1697;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52826;}i:1698;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:"id";}i:2;i:52827;}i:1699;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52829;}i:1700;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:": 10,
      ";}i:2;i:52830;}i:1701;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52842;}i:1702;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:4:"name";}i:2;i:52843;}i:1703;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52847;}i:1704;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:52848;}i:1705;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52850;}i:1706;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:11:"Helen White";}i:2;i:52851;}i:1707;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52862;}i:1708;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:8:",
      ";}i:2;i:52863;}i:1709;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52871;}i:1710;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"contact";}i:2;i:52872;}i:1711;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52879;}i:1712;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:": {
        ";}i:2;i:52880;}i:1713;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52892;}i:1714;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:"address";}i:2;i:52893;}i:1715;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52900;}i:1716;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:14:": {
          ";}i:2;i:52901;}i:1717;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52915;}i:1718;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"street";}i:2;i:52916;}i:1719;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52922;}i:1720;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:52923;}i:1721;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52925;}i:1722;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:13:"852 Spruce St";}i:2;i:52926;}i:1723;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52939;}i:1724;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:",
          ";}i:2;i:52940;}i:1725;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52952;}i:1726;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:4:"city";}i:2;i:52953;}i:1727;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52957;}i:1728;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:52958;}i:1729;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52960;}i:1730;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:8:"Lasttown";}i:2;i:52961;}i:1731;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52969;}i:1732;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:",
          ";}i:2;i:52970;}i:1733;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52982;}i:1734;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"state";}i:2;i:52983;}i:1735;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52988;}i:1736;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:52989;}i:1737;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:52991;}i:1738;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:"VA";}i:2;i:52992;}i:1739;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:52994;}i:1740;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:",
          ";}i:2;i:52995;}i:1741;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:53007;}i:1742;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"zip";}i:2;i:53008;}i:1743;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:53011;}i:1744;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:53012;}i:1745;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:53014;}i:1746;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"96321";}i:2;i:53015;}i:1747;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:53020;}i:1748;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:20:"
        },
        ";}i:2;i:53021;}i:1749;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:53041;}i:1750;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"phone";}i:2;i:53042;}i:1751;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:53047;}i:1752;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:53048;}i:1753;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:53050;}i:1754;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:"555-555-7890";}i:2;i:53051;}i:1755;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:53063;}i:1756;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:10:",
        ";}i:2;i:53064;}i:1757;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:53074;}i:1758;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:"email";}i:2;i:53075;}i:1759;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:53080;}i:1760;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:": ";}i:2;i:53081;}i:1761;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:53083;}i:1762;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:22:"helenwhite@example.com";}i:2;i:53084;}i:1763;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:53106;}i:1764;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:177:"
      }
    }
  ]
}
~~~
</details>

If using nested `JSON` winds up being too verbose for your token budget, fallback to `relational tables` defined with `Markdown`:

<p align=";}i:2;i:53107;}i:1765;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:53284;}i:1766;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"center";}i:2;i:53285;}i:1767;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:53291;}i:1768;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:">
  <img width=";}i:2;i:53292;}i:1769;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:53307;}i:1770;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"550";}i:2;i:53308;}i:1771;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:53311;}i:1772;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:" src=";}i:2;i:53312;}i:1773;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:53317;}i:1774;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/233507968-a378587b-e468-4882-a1e8-678d9f3933d3.png";i:1;N;}i:2;i:53318;}i:1775;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:53416;}i:1776;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:" title=";}i:2;i:53417;}i:1777;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:53424;}i:1778;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:52:"GPT-4 handles relational tables pretty reliably too.";}i:2;i:53425;}i:1779;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:53477;}i:1780;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:198:">
</p>

<details>
<summary>(Full prompt)</summary>

~~~
You are a helpful assistant. You answer questions about users. Here is what you know about them:

Table 1: users
| id (PK) | name          |
|";}i:2;i:53478;}i:1781;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:53676;}i:1782;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:53679;}i:1783;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:53682;}i:1784;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1:"|";}i:2;i:53685;}i:1785;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:53686;}i:1786;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:53689;}i:1787;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:53692;}i:1788;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:53695;}i:1789;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:53698;}i:1790;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:377:"|
| 1       | John Doe      |
| 2       | Jane Smith    |
| 3       | Alice Johnson |
| 4       | Bob Williams  |
| 5       | Charlie Brown |
| 6       | Diane Davis   |
| 7       | Edward Martinez |
| 8       | Fiona Taylor  |
| 9       | George Thompson |
| 10      | Helen White   |

Table 2: addresses
| id (PK) | user_id (FK) | street      | city       | state | zip   |
|";}i:2;i:53701;}i:1791;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:54078;}i:1792;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:54081;}i:1793;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:54084;}i:1794;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1:"|";}i:2;i:54087;}i:1795;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:54088;}i:1796;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:54091;}i:1797;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:54094;}i:1798;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:54097;}i:1799;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:2:"--";}i:2;i:54100;}i:1800;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1:"|";}i:2;i:54102;}i:1801;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:54103;}i:1802;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:54106;}i:1803;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:54109;}i:1804;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:54112;}i:1805;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:"-|";}i:2;i:54115;}i:1806;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:54117;}i:1807;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:54120;}i:1808;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:54123;}i:1809;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:54126;}i:1810;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1:"|";}i:2;i:54129;}i:1811;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:54130;}i:1812;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:54133;}i:1813;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:"-|";}i:2;i:54136;}i:1814;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:54138;}i:1815;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:54141;}i:1816;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:769:"-|
| 1       | 1            | 123 Main St | Anytown    | CA    | 12345 |
| 2       | 2            | 456 Elm St  | Sometown   | TX    | 54321 |
| 3       | 3            | 789 Oak St  | Othertown  | NY    | 67890 |
| 4       | 4            | 135 Maple St | Thistown  | FL    | 98765 |
| 5       | 5            | 246 Pine St | Thatstown  | WA    | 86420 |
| 6       | 6            | 369 Willow St | Sumtown  | CO    | 15980 |
| 7       | 7            | 482 Aspen St | Newtown   | MI    | 35742 |
| 8       | 8            | 531 Birch St | Oldtown   | OH    | 85249 |
| 9       | 9            | 678 Cedar St | Nexttown  | GA    | 74125 |
| 10      | 10           | 852 Spruce St | Lasttown | VA    | 96321 |

Table 3: phone_numbers
| id (PK) | user_id (FK) | phone       |
|";}i:2;i:54144;}i:1817;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:54913;}i:1818;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:54916;}i:1819;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:54919;}i:1820;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1:"|";}i:2;i:54922;}i:1821;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:54923;}i:1822;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:54926;}i:1823;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:54929;}i:1824;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:54932;}i:1825;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:2:"--";}i:2;i:54935;}i:1826;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1:"|";}i:2;i:54937;}i:1827;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:54938;}i:1828;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:54941;}i:1829;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:54944;}i:1830;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:54947;}i:1831;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:492:"-|
| 1       | 1            | 555-555-1234 |
| 2       | 2            | 555-555-5678 |
| 3       | 3            | 555-555-2468 |
| 4       | 4            | 555-555-8642 |
| 5       | 5            | 555-555-7531 |
| 6       | 6            | 555-555-9512 |
| 7       | 7            | 555-555-6813 |
| 8       | 8            | 555-555-4268 |
| 9       | 9            | 555-555-3142 |
| 10      | 10           | 555-555-7890 |

Table 4: emails
| id (PK) | user_id (FK) | email                 |
|";}i:2;i:54950;}i:1832;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:55442;}i:1833;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:55445;}i:1834;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:55448;}i:1835;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1:"|";}i:2;i:55451;}i:1836;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:55452;}i:1837;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:55455;}i:1838;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:55458;}i:1839;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:55461;}i:1840;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:2:"--";}i:2;i:55464;}i:1841;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1:"|";}i:2;i:55466;}i:1842;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:55467;}i:1843;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:55470;}i:1844;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:55473;}i:1845;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:55476;}i:1846;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:55479;}i:1847;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:55482;}i:1848;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:55485;}i:1849;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:2:"--";}i:2;i:55488;}i:1850;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:616:"|
| 1       | 1            | johndoe@example.com   |
| 2       | 2            | janesmith@example.com |
| 3       | 3            | alicejohnson@example.com |
| 4       | 4            | bobwilliams@example.com |
| 5       | 5            | charliebrown@example.com |
| 6       | 6            | dianedavis@example.com |
| 7       | 7            | edwardmartinez@example.com |
| 8       | 8            | fionataylor@example.com |
| 9       | 9            | georgethompson@example.com |
| 10      | 10           | helenwhite@example.com |

Table 5: cities
| id (PK) | name         | state | population | median_income |
|";}i:2;i:55490;}i:1851;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:56106;}i:1852;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:56109;}i:1853;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:56112;}i:1854;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1:"|";}i:2;i:56115;}i:1855;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:56116;}i:1856;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:56119;}i:1857;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:56122;}i:1858;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:56125;}i:1859;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:2:"--";}i:2;i:56128;}i:1860;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1:"|";}i:2;i:56130;}i:1861;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:56131;}i:1862;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:56134;}i:1863;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:2:"-|";}i:2;i:56137;}i:1864;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:56139;}i:1865;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:56142;}i:1866;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:56145;}i:1867;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:56148;}i:1868;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:1:"|";}i:2;i:56151;}i:1869;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:56152;}i:1870;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:56155;}i:1871;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:56158;}i:1872;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:56161;}i:1873;a:3:{i:0;s:6:"entity";i:1;a:1:{i:0;s:3:"---";}i:2;i:56164;}i:1874;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:698:"|
| 1       | Anytown     | CA    | 50,000     | $70,000      |
| 2       | Sometown    | TX    | 100,000    | $60,000      |
| 3       | Othertown   | NY    | 25,000     | $80,000      |
| 4       | Thistown    | FL    | 75,000     | $65,000      |
| 5       | Thatstown   | WA    | 40,000     | $75,000      |
| 6       | Sumtown     | CO    | 20,000     | $85,000      |
| 7       | Newtown     | MI    | 60,000     | $55,000      |
| 8       | Oldtown     | OH    | 30,000     | $70,000      |
| 9       | Nexttown    | GA    | 15,000     | $90,000      |
| 10      | Lasttown    | VA    | 10,000     | $100,000     |
~~~

</details>

> 🧠 The model works well with data in [3rd normal form](";}i:2;i:56167;}i:1875;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:47:"https://en.wikipedia.org/wiki/Third_normal_form";i:1;N;}i:2;i:56865;}i:1876;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:616:"), but may struggle with too many joins. In experiments, it seems to do okay with at least three levels of nested joins. In the example above the model successfully joins from `users` to `addresses` to `cities` to infer the likely income for George – $90,000.

### Citations

Frequently, a natural language response isn’t sufficient on its own and you’ll want the model’s output to cite where it is getting data from. 

One useful thing to note here is that anything you might want to cite should have a unique ID. The simplest approach is to just ask the model to link to anything it references:


<p align=";}i:2;i:56912;}i:1877;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:57528;}i:1878;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"center";}i:2;i:57529;}i:1879;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:57535;}i:1880;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:">
  <img width=";}i:2;i:57536;}i:1881;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:57551;}i:1882;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"550";}i:2;i:57552;}i:1883;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:57555;}i:1884;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:" src=";}i:2;i:57556;}i:1885;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:57561;}i:1886;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/233509069-1dcbffa2-8357-49b5-be43-9791f93bd0f8.png";i:1;N;}i:2;i:57562;}i:1887;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:57660;}i:1888;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:" title=";}i:2;i:57661;}i:1889;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:57668;}i:1890;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:50:"GPT-4 will reliably link to data if you ask it to.";}i:2;i:57669;}i:1891;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:57719;}i:1892;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:594:">
</p>

### Programmatic Consumption

By default, language models output natural language text, but frequently we need to interact with this result in a programmatic way that goes beyond simply printing it out on screen. You can achieve this by  asking the model to output the results in your favorite serialization format (JSON and YAML seem to work best).

Make sure you give the model an example of the output format you’d like. Building on our previous travel example above, we can augment our prompt to tell it:

~~~
Produce your output as JSON. The format should be:
```
{
    message: ";}i:2;i:57720;}i:1893;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:58314;}i:1894;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:28:"The message to show the user";}i:2;i:58315;}i:1895;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:58343;}i:1896;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:144:",
    hotelId: 432,
    flightId: 831
}
```

Do not include the IDs in your message.
~~~

And now we’ll get interactions like this:

<p align=";}i:2;i:58344;}i:1897;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:58488;}i:1898;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"center";}i:2;i:58489;}i:1899;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:58495;}i:1900;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:">
  <img width=";}i:2;i:58496;}i:1901;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:58511;}i:1902;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"550";}i:2;i:58512;}i:1903;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:58515;}i:1904;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:" src=";}i:2;i:58516;}i:1905;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:58521;}i:1906;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/233509174-be0c3bc5-08e3-4d1a-8841-52c401def770.png";i:1;N;}i:2;i:58522;}i:1907;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:58620;}i:1908;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:" title=";}i:2;i:58621;}i:1909;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:58628;}i:1910;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:70:"GPT-4 providing travel recommendations in an easy to work with format.";}i:2;i:58629;}i:1911;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:58699;}i:1912;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:566:">
</p>

You could imagine the UI for this rendering the message as normal text, but then also adding discrete buttons for booking the flight + hotel, or auto-filling a form for the user.

As another example, let’s build on the [citations](#citations) example – but move beyond Markdown links. We can ask it to produce JSON with a normal message along with a list of items used in the creation of that message. In this scenario you won’t know exactly where in the message the citations were leveraged, but you’ll know that they were used somewhere.

<p align=";}i:2;i:58700;}i:1913;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:59266;}i:1914;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"center";}i:2;i:59267;}i:1915;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:59273;}i:1916;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:">
  <img width=";}i:2;i:59274;}i:1917;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:59289;}i:1918;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"550";}i:2;i:59290;}i:1919;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:59293;}i:1920;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:" src=";}i:2;i:59294;}i:1921;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:59299;}i:1922;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/233509280-59d9ff46-0e95-488a-b314-a7d2b7c9bfa3.png";i:1;N;}i:2;i:59300;}i:1923;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:59398;}i:1924;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:" title=";}i:2;i:59399;}i:1925;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:59406;}i:1926;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:137:"Asking the model to provide a list of citations is a reliable way to programmatically know what data the model leaned on in its response.";}i:2;i:59407;}i:1927;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:59544;}i:1928;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:138:">
</p>

> 🧠 Interestingly, in the model’s response to “How much did I spend at Target?” it provides a single value, $188.16, but ";}i:2;i:59545;}i:1929;a:3:{i:0;s:11:"strong_open";i:1;a:0:{}i:2;i:59683;}i:1930;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:11:"importantly";}i:2;i:59685;}i:1931;a:3:{i:0;s:12:"strong_close";i:1;a:0:{}i:2;i:59696;}i:1932;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:590:" in the `citations` array it lists the individual expenses that it used to compute that value.

### Chain of Thought

Sometimes you will bang your head on a prompt trying to get the model to output reliable results, but, no matter what you do, it just won’t work. This will frequently happen when the bot’s final output requires intermediate thinking, but you ask the bot only for the output and nothing else.

The answer may surprise you: ask the bot to show its work. In October 2022, Google released a paper “[Chain-of-Thought Prompting Elicits Reasoning in Large Language Models](";}i:2;i:59698;}i:1933;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:36:"https://arxiv.org/pdf/2201.11903.pdf";i:1;N;}i:2;i:60288;}i:1934;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:412:")” where they showed that if, in your hidden prompt, you give the bot examples of answering questions by showing your work, then when you ask the bot to answer something it will show its work and produce more reliable answers.

Just a few weeks after that paper was published, at the end of October 2022, the University of Tokyo and Google released the paper “[Large Language Models are Zero-Shot Reasoners](";}i:2;i:60324;}i:1935;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:41:"https://openreview.net/pdf?id=e2TBb5y0yFf";i:1;N;}i:2;i:60736;}i:1936;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:73:")”, where they show that you don’t even need to provide examples – ";}i:2;i:60777;}i:1937;a:3:{i:0;s:11:"strong_open";i:1;a:0:{}i:2;i:60850;}i:1938;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:52:"you simply have to ask the bot to think step-by-step";}i:2;i:60852;}i:1939;a:3:{i:0;s:12:"strong_close";i:1;a:0:{}i:2;i:60904;}i:1940;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:197:".

#### Averaging

Here is an example where we ask the bot to compute the average expense, excluding Target. The actual answer is $136.77 and the bot almost gets it correct with $136.43.

<p align=";}i:2;i:60906;}i:1941;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:61103;}i:1942;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"center";}i:2;i:61104;}i:1943;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:61110;}i:1944;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:">
  <img width=";}i:2;i:61111;}i:1945;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:61126;}i:1946;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"550";}i:2;i:61127;}i:1947;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:61130;}i:1948;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:" src=";}i:2;i:61131;}i:1949;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:61136;}i:1950;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/233509534-2b32c8dd-a1ee-42ea-82fb-4f84cfe7e9ba.png";i:1;N;}i:2;i:61137;}i:1951;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:61235;}i:1952;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:" title=";}i:2;i:61236;}i:1953;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:61243;}i:1954;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:10:"The model ";}i:2;i:61244;}i:1955;a:3:{i:0;s:11:"strong_open";i:1;a:0:{}i:2;i:61254;}i:1956;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"almost";}i:2;i:61256;}i:1957;a:3:{i:0;s:12:"strong_close";i:1;a:0:{}i:2;i:61262;}i:1958;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:50:" gets the average correct, but is a few cents off.";}i:2;i:61264;}i:1959;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:61314;}i:1960;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:104:">
</p>

If we simply add “Let’s think step-by-step”, the model gets the correct answer:

<p align=";}i:2;i:61315;}i:1961;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:61419;}i:1962;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"center";}i:2;i:61420;}i:1963;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:61426;}i:1964;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:">
  <img width=";}i:2;i:61427;}i:1965;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:61442;}i:1966;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"550";}i:2;i:61443;}i:1967;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:61446;}i:1968;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:" src=";}i:2;i:61447;}i:1969;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:61452;}i:1970;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/233509608-6e53995b-668b-47f6-9b5e-67afad89f8bc.png";i:1;N;}i:2;i:61453;}i:1971;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:61551;}i:1972;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:" title=";}i:2;i:61552;}i:1973;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:61559;}i:1974;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:67:"When we ask the model to show its work, it gets the correct answer.";}i:2;i:61560;}i:1975;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:61627;}i:1976;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:414:">
</p>

#### Interpreting Code

Let’s revisit the Python example from earlier and apply chain-of-thought prompting to our question. As a reminder, when we asked the bot to evaluate the Python code it gets it slightly wrong. The correct answer is `Hello, Brex!!Brex!!Brex!!!` but the bot gets confused about the number of !'s to include. In below’s example, it outputs `Hello, Brex!!!Brex!!!Brex!!!`:

<p align=";}i:2;i:61628;}i:1977;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:62042;}i:1978;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"center";}i:2;i:62043;}i:1979;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:62049;}i:1980;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:">
  <img width=";}i:2;i:62050;}i:1981;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:62065;}i:1982;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"550";}i:2;i:62066;}i:1983;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:62069;}i:1984;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:" src=";}i:2;i:62070;}i:1985;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:62075;}i:1986;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/233509724-8f3302f8-59eb-4d3b-8939-53d7f63b0299.png";i:1;N;}i:2;i:62076;}i:1987;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:62174;}i:1988;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:" title=";}i:2;i:62175;}i:1989;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:62182;}i:1990;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:73:"The bot almost interprets the Python code correctly, but is a little off.";}i:2;i:62183;}i:1991;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:62256;}i:1992;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:87:">
</p>

If we ask the bot to show its work, then it gets the correct answer:

<p align=";}i:2;i:62257;}i:1993;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:62344;}i:1994;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"center";}i:2;i:62345;}i:1995;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:62351;}i:1996;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:">
  <img width=";}i:2;i:62352;}i:1997;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:62367;}i:1998;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"550";}i:2;i:62368;}i:1999;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:62371;}i:2000;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:" src=";}i:2;i:62372;}i:2001;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:62377;}i:2002;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/233509790-2a0f2189-d864-4d27-aacb-cfc936fad907.png";i:1;N;}i:2;i:62378;}i:2003;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:62476;}i:2004;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:" title=";}i:2;i:62477;}i:2005;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:62484;}i:2006;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:76:"The bot correctly interprets the Python code if you ask it to show its work.";}i:2;i:62485;}i:2007;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:62561;}i:2008;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:316:">
</p>

#### Delimiters

In many scenarios, you may not want to show the end user all of the bot’s thinking and instead just want to show the final answer. You can ask the bot to delineate the final answer from its thinking. There are many ways to do this, but let’s use JSON to make it easy to parse:

<p align=";}i:2;i:62562;}i:2009;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:62878;}i:2010;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:6:"center";}i:2;i:62879;}i:2011;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:62885;}i:2012;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:15:">
  <img width=";}i:2;i:62886;}i:2013;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:62901;}i:2014;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:3:"550";}i:2;i:62902;}i:2015;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:62905;}i:2016;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:5:" src=";}i:2;i:62906;}i:2017;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:62911;}i:2018;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:98:"https://user-images.githubusercontent.com/89960/233509865-4f3e7265-6645-4d43-8644-ecac5c0ca4a7.png";i:1;N;}i:2;i:62912;}i:2019;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:63010;}i:2020;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:7:" title=";}i:2;i:63011;}i:2021;a:3:{i:0;s:18:"doublequoteopening";i:1;a:0:{}i:2;i:63018;}i:2022;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:84:"The bot showing its work while also delimiting the final answer for easy extraction.";}i:2;i:63019;}i:2023;a:3:{i:0;s:18:"doublequoteclosing";i:1;a:0:{}i:2;i:63103;}i:2024;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:423:">
</p>

Using Chain-of-Thought prompting will consume more tokens, resulting in increased price and latency, but the results are noticeably more reliable for many scenarios. It’s a valuable tool to use when you need the bot to do something complex and as reliably as possible.

### Fine Tuning

Sometimes no matter what tricks you throw at the model, it just won’t do what you want it to do. In these scenarios you can ";}i:2;i:63104;}i:2025;a:3:{i:0;s:11:"strong_open";i:1;a:0:{}i:2;i:63527;}i:2026;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:9:"sometimes";}i:2;i:63529;}i:2027;a:3:{i:0;s:12:"strong_close";i:1;a:0:{}i:2;i:63538;}i:2028;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:84:" fallback to fine-tuning. This should, in general, be a last resort.

[Fine-tuning](";}i:2;i:63540;}i:2029;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:51:"https://platform.openai.com/docs/guides/fine-tuning";i:1;N;}i:2;i:63624;}i:2030;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:459:") is the process of taking an already trained model and then giving it thousands (or more) of example `input:output` pairs

It does not eliminate the need for hidden prompts, because you still need to embed dynamic data, but it may make the prompts smaller and more reliable.

#### Downsides

There are many downsides to fine-tuning. If it is at all possible, take advantage of the nature of language models being [zero-shot, one-shot, and few-shot learners](";}i:2;i:63675;}i:2031;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:48:"https://en.wikipedia.org/wiki/Few-shot_learning_";i:1;N;}i:2;i:64134;}i:2032;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:140:"(natural_language_processing)) by teaching them to do something in their prompt rather than fine-tuning.

Some of the downsides include:

- ";}i:2;i:64182;}i:2033;a:3:{i:0;s:11:"strong_open";i:1;a:0:{}i:2;i:64322;}i:2034;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:12:"Not possible";}i:2;i:64324;}i:2035;a:3:{i:0;s:12:"strong_close";i:1;a:0:{}i:2;i:64336;}i:2036;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:39:": [GPT-3.5/GPT-4 isn’t fine tunable](";}i:2;i:64338;}i:2037;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:87:"https://platform.openai.com/docs/guides/chat/is-fine-tuning-available-for-gpt-3-5-turbo";i:1;N;}i:2;i:64377;}i:2038;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:32:"), which is the primary model / ";}i:2;i:64464;}i:2039;a:3:{i:0;s:7:"acronym";i:1;a:1:{i:0;s:3:"API";}i:2;i:64496;}i:2040;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:63:" we’ll be using, so we simply can’t lean in fine-tuning.
- ";}i:2;i:64499;}i:2041;a:3:{i:0;s:11:"strong_open";i:1;a:0:{}i:2;i:64562;}i:2042;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:8:"Overhead";}i:2;i:64564;}i:2043;a:3:{i:0;s:12:"strong_close";i:1;a:0:{}i:2;i:64572;}i:2044;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:57:": Fine-tuning requires manually creating tons of data.
- ";}i:2;i:64574;}i:2045;a:3:{i:0;s:11:"strong_open";i:1;a:0:{}i:2;i:64631;}i:2046;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:8:"Velocity";}i:2;i:64633;}i:2047;a:3:{i:0;s:12:"strong_close";i:1;a:0:{}i:2;i:64641;}i:2048;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:250:": The iteration loop becomes much slower – every time you want to add a new capability, instead of adding a few lines to a prompt, you need to create a bunch of fake data and then run the finetune process and then use the newly fine-tuned model.
- ";}i:2;i:64643;}i:2049;a:3:{i:0;s:11:"strong_open";i:1;a:0:{}i:2;i:64893;}i:2050;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:4:"Cost";}i:2;i:64895;}i:2051;a:3:{i:0;s:12:"strong_close";i:1;a:0:{}i:2;i:64899;}i:2052;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:223:": It is up to 60x more expensive to use a fine-tuned GPT-3 model vs the stock `gpt-3.5-turbo` model. And it is 2x more expensive to use a fine-tuned GPT-3 model vs the stock GPT-4 model.

> ⛔️ If you fine-tune a model, ";}i:2;i:64901;}i:2053;a:3:{i:0;s:11:"strong_open";i:1;a:0:{}i:2;i:65124;}i:2054;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:28:"never use real customer data";}i:2;i:65126;}i:2055;a:3:{i:0;s:12:"strong_close";i:1;a:0:{}i:2;i:65154;}i:2056;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:326:". Always use synthetic data. The model may memorize portions of the data you provide and may regurgitate private data to other users that shouldn’t be seeing it.
>
> If you never fine-tune a model, we don’t have to worry about accidentally leaking data into the model.

## Additional Resources
- :star2: [OpenAI Cookbook](";}i:2;i:65156;}i:2057;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:41:"https://github.com/openai/openai-cookbook";i:1;N;}i:2;i:65482;}i:2058;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:44:") :star2:
- :technologist: [Prompt Hacking](";}i:2;i:65523;}i:2059;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:56:"https://learnprompting.org/docs/category/-prompt-hacking";i:1;N;}i:2;i:65567;}i:2060;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:63:") :technologist: 
- :books: [Dair.ai Prompt Engineering Guide](";}i:2;i:65623;}i:2061;a:3:{i:0;s:12:"externallink";i:1;a:2:{i:0;s:51:"https://github.com/dair-ai/Prompt-Engineering-Guide";i:1;N;}i:2;i:65686;}i:2062;a:3:{i:0;s:5:"cdata";i:1;a:1:{i:0;s:10:") :books: ";}i:2;i:65737;}i:2063;a:3:{i:0;s:7:"p_close";i:1;a:0:{}i:2;i:65737;}i:2064;a:3:{i:0;s:12:"document_end";i:1;a:0:{}i:2;i:65737;}}