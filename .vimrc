" sudo apt install vim-gtk3 # to use clipboard
filetype plugin indent on

syntax on

inoremap jj <Esc>
set number
set clipboard=unnamedplus

set shiftwidth=2
set tabstop=2
set expandtab
set softtabstop=2
set list listchars=tab:\▸\-

set title
set titlestring=%f

nnoremap <C-F5> :w<CR>:!python3 %<CR>
" 補完をctrl @に割り当て
inoremap <C-@> <C-p>

" Emacs風キーバインド（インサートモード）
" 行の先頭に移動
inoremap <C-a> <C-o>^
" 行の末尾に移動
inoremap <C-e> <C-o>$
" 左に移動
inoremap <C-b> <Left>
" 右に移動
inoremap <C-f> <Right>
" 上に移動
inoremap <C-p> <Up>
" 下に移動
"inoremap <C-n> <Down>
" 1文字削除（後ろ）
inoremap <C-d> <Del>
" 1文字削除（前）
inoremap <C-h> <BS>
" カーソルから行末まで削除
inoremap <C-k> <C-o>D
" ヤンク（ペースト）
inoremap <C-y> <C-r>+


augroup python_indent
    autocmd!
    autocmd FileType python setlocal shiftwidth=4
    autocmd FileType python setlocal tabstop=4
    autocmd FileType python setlocal softtabstop=4
    autocmd FileType python setlocal expandtab
augroup END

augroup cpp_indent
    autocmd!
    autocmd FileType cpp setlocal shiftwidth=2
    autocmd FileType cpp setlocal tabstop=2
    autocmd FileType cpp setlocal softtabstop=2
    autocmd FileType cpp setlocal expandtab
augroup END

augroup verilog_indent
    autocmd!
    autocmd FileType verilog,systemverilog setlocal shiftwidth=2
    autocmd FileType verilog,systemverilog setlocal tabstop=2
    autocmd FileType verilog,systemverilog setlocal softtabstop=2
    autocmd FileType verilog,systemverilog setlocal expandtab
augroup END

augroup makefile_indent
    autocmd!
    autocmd FileType make setlocal noexpandtab    
    autocmd FileType make setlocal tabstop=8     
    autocmd FileType make setlocal shiftwidth=8 
    autocmd FileType make setlocal softtabstop=0
augroup END

augroup html_indent
    autocmd!
    autocmd FileType html setlocal shiftwidth=2
    autocmd FileType html setlocal tabstop=2
    autocmd FileType html setlocal softtabstop=2
    autocmd FileType html setlocal expandtab
augroup END

augroup javascript_indent
    autocmd!
    autocmd FileType javascript setlocal shiftwidth=2
    autocmd FileType javascript setlocal tabstop=2
    autocmd FileType javascript setlocal softtabstop=2
    autocmd FileType javascript setlocal expandtab
augroup END


augroup csharp_indent
    autocmd!
    autocmd FileType cs setlocal shiftwidth=4
    autocmd FileType cs setlocal tabstop=4
    autocmd FileType cs setlocal softtabstop=4
    autocmd FileType cs setlocal expandtab
augroup END
