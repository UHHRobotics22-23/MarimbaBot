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
                c'4
                d'4
                e'4
                f'4
                g'4
                a'4
                b'4
                c''4
            }
        }
    >>
    \midi {}
%! abjad.LilyPondFile._get_formatted_blocks()
}