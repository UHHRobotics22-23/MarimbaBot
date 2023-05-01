%! abjad.LilyPondFile._get_format_pieces()
\version "2.20.0"
%! abjad.LilyPondFile._get_format_pieces()
\language "english"

%! abjad.LilyPondFile._get_formatted_blocks()
\score
%! abjad.LilyPondFile._get_formatted_blocks()
{
    \context Score = "Score"
    <<
        \context Staff = "RH_Staff"
        {
            \context Voice = "RH_Voice"
            {
                d'8
                f'8
                a'8
                d''8
                f''8
                gs'4
                r8
                e'8
                gs'8
                b'8
                e''8
                gs''8
                a'4
            }
        }
    >>
    \midi {}
%! abjad.LilyPondFile._get_formatted_blocks()
}