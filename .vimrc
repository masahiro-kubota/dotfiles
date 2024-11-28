" sudo apt install vim-gtk3 # to use clipboard
filetype plugin indent on


inoremap jj <Esc>
set number
set clipboard=unnamedplus

set shiftwidth=4
set tabstop=4
set expandtab
set softtabstop=4
set list listchars=tab:\▸\-

set title
set titlestring=%f

nnoremap <C-F5> :w<CR>:!python3 %<CR>

" Emacs風キーバインド（インサートモード）
inoremap <C-a> <C-o>^   " 行の先頭に移動
inoremap <C-e> <C-o>$   " 行の末尾に移動
inoremap <C-b> <Left>   " 左に移動
inoremap <C-f> <Right>  " 右に移動
inoremap <C-p> <Up>     " 上に移動
inoremap <C-n> <Down>   " 下に移動
inoremap <C-d> <Del>    " 1文字削除（後ろ）
inoremap <C-h> <BS>     " 1文字削除（前）
inoremap <C-k> <C-o>D   " カーソルから行末まで削除
inoremap <C-y> <C-r>+   " ヤンク（ペースト）

augroup python_indent
    autocmd!
    autocmd FileType python setlocal shiftwidth=4
    autocmd FileType python setlocal tabstop=4
    autocmd FileType python setlocal softtabstop=4
    autocmd FileType python setlocal expandtab
augroup END

augroup verilog_indent
    autocmd!
    autocmd FileType verilog,systemverilog setlocal shiftwidth=2
    autocmd FileType verilog,systemverilog setlocal tabstop=2
    autocmd FileType verilog,systemverilog setlocal softtabstop=2
    autocmd FileType verilog,systemverilog setlocal expandtab
augroup END

