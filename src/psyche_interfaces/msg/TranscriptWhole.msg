# High-quality diff emitted by ``asr_hq`` with replacements applied to the
# canonical transcript buffer. Each index pair defines the half-open character
# range ``[start_char[i], end_char[i])`` to be substituted with ``text[i]``.
builtin_interfaces/Time stamp_emitted # when the refinement was produced
uint32[] start_char                   # inclusive starting offsets in the document
uint32[] end_char                     # exclusive end offsets for each replacement
string[] text                         # replacement strings in UTF-8
