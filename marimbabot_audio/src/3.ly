%! abjad.LilyPondFile._get_format_pieces()
\version "2.20.0"
%! abjad.LilyPondFile._get_format_pieces()
\language "english"

%! abjad.LilyPondFile._get_formatted_blocks()
\header
%! abjad.LilyPondFile._get_formatted_blocks()
{
    tagline = #ff
%! abjad.LilyPondFile._get_formatted_blocks()
}
%! abjad.LilyPondFile._get_formatted_blocks()
\score
%! abjad.LilyPondFile._get_formatted_blocks()
{
    \context Score = "Score"
    <<
        \new Staff
        {
            \new Voice
            {
                c4
                e4
                f4
            }
        }
    >>
    \midi {}
%! abjad.LilyPondFile._get_formatted_blocks()
}