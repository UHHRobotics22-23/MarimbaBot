import os
from pynini.lib import pynutil
import pynini
# sorry for forget to exchange the number position, dreiundzwangzig -> 32, but it should be 23.
def apply_fst(text, fst):
	"""
	Given a string input, returns the output string
	produced by traversing the path with lowest weight.
	If no valid path accepts input string, returns an
	error
	"""
	try:
		lattice = text @ fst
		output_text = pynini.shortestpath(lattice, nshortest=1, unique=True).string()
		# print(pynini.shortestpath(text @ fst).string())
		print(output_text)
	except:
		print(f"Error: No valid output with given input: '{text}'")

def run():
	file_abs_path = os.path.dirname(os.path.abspath(__file__))
	data_root_path = os.path.join(file_abs_path,'data')
	number_root = os.path.join(data_root_path,'numbers')
	pth = os.path.join(number_root,'digit.tsv')
	digit_bt_2 = pynini.string_file(pth)
	digit_1 = pynini.string_file(os.path.join(number_root,'ones.tsv'))
	digit = digit_1 | digit_bt_2
	teens = pynini.string_file(os.path.join(number_root,'teen.tsv'))
	ties = pynini.string_file(os.path.join(number_root,'ties.tsv'))

	#  21,22,23,...,29,31,...39,41,....99 (not include 10,20,30...)
	delete_und = pynini.closure(pynutil.delete("und"), 0, 1)
	graph_digit_ties = digit + delete_und + ties

	# 10,20,30,40 ...
	add_zero = pynutil.insert("0")
	graph_ties = ties + add_zero

	# final graph
	graph_number = graph_digit_ties | digit | teens | graph_ties

	# use optimizer to reduce the size of parameters for efficiency
	print(f"number of states:{graph_number.num_states()}")
	graph_number.optimize()
	print(f"number of states:{graph_number.num_states()}")

	# show the results, sry, forget to inverse the position of the first digit and second digit
	apply_fst("dreiundzwanzig",graph_number)
	apply_fst("vierundzwanzig",graph_number)
	apply_fst("vierzig",graph_number)
	apply_fst("siebzig",graph_number)
	apply_fst("siebenundsiebzig",graph_number)

if __name__ == '__main__':
	run()